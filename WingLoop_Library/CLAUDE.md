# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

WingLoop is a Python framework that gives [ASWING](https://web.mit.edu/drela/Public/web/aswing/) (a Fortran
structural/aerodynamic aircraft analysis tool) closed-loop control capability. ASWING natively only supports
linear bi-scheduled controllers; WingLoop drives ASWING as a subprocess, exchanging state/control data through
text files each timestep, so that arbitrary control laws — written in Python, MATLAB, or Simulink — can be
applied during a time-transient simulation.

This directory (`WingLoop_Library/`) is the installable package, one level below the repo root (which also
holds `pyproject.toml`, `README.md`, `LICENCE`, `CONTRIBUTING.md`). The package is installed as `WingLoop_Library`
(see `pyproject.toml` at repo root), not as the repo name `WingLoop`.

Reference publication: Avoni, Bronz, Condomines, Moschetta, "Enhancing ASWING Flight Dynamics Simulations with
Closed-Loop Control for Flexible Aircraft," AIAA 2025-3425, 2025.

## License / contribution model

CC BY-NC-SA 4.0 (non-commercial). Contributions require agreeing to the CLA described in `CONTRIBUTING.md`
(contributor grants the author broad rights, including for a future commercial license) — record agreement by
adding your name to `CONTRIBUTORS.md`. Keep the copyright/license header block at the top of every source file
intact when editing.

## Setup and running

Install in editable mode from the repo root (one level up from this directory):

```bash
pip install -e .
```

Requires Python `>=3.8,<3.13`. Dependencies: `numpy`, `matplotlib`, `fmpy`, `inotify_simple`, `matlabengine`
(only needed for MATLAB/Simulink controller backends).

There is no unit test suite / CI — validation is done by running the end-to-end example against a real ASWING
install:

```bash
cd wingloop_testrun
python wingloop_testrun.py
```

This requires ASWING itself to be installed and resolvable (either an explicit path passed to `Launch_ASWING`,
or an `aswing_alias` shell alias resolved via `bash -ic 'type <alias>'`). Without ASWING present, this script
cannot run to completion — code changes affecting `Aswing_Director`/`WingLoop` orchestration logic generally
cannot be exercised without it.

Individual modules also have their own `if __name__ == "__main__":` demo/test blocks (e.g. `Aswing_Director.py`,
`PyControl_IO.py`) that can be run standalone for quicker iteration on isolated pieces (file parsing, command
send/receive) without a full simulation.

In `wingloop_testrun.py`, the `selector` variable (`"py"`, `"mat"`, `"sim"`, `"fmu"`) switches which controller
backend/example is exercised.

## Architecture

Execution is a **time-marching loop split across two processes**: ASWING (Fortran, subprocess) and Python.
Each iteration: ASWING writes its current state to a text file → Python parses it → a control law computes new
actuator commands → Python writes them to a text file → ASWING reads it and advances `K` timesteps → repeat.

### Module responsibilities

- **`WingLoop.py`** — top-level orchestrator (`WingLoop` class). Owns the whole run: launches ASWING, sets up
  plotting, drives `Time_Transient_Simulation` (which loops in chunks of `K` iterations via
  `Performing_K_iterations_ASWING`), and handles output/shutdown. This is the class user scripts instantiate.
- **`Aswing_Director.py`** — low-level subprocess interface to the ASWING executable (`Aswing_Director` class).
  Sends commands over stdin, drains stdout/stderr via background threads + queues. Two families of methods:
  - `send_command_and_receive` — fire-and-forget with a fixed wait window, **no real synchronization guarantee**
    with ASWING's actual progress.
  - `send_writefile_command_and_receive` — used for the per-timestep state write; blocks on an `inotify` watch
    for `CLOSE_WRITE` on the expected output file, so it's the one place real sync happens. An older polling
    based implementation (`..._old`) is kept for the "delete" file-write strategy in `WingLoop.py`.
  - If `aswing_path` is not given, the executable is resolved from a bash alias (`aswing_alias`) via
    `type <alias>` — ASWING is expected to be aliased in the user's shell, not on `PATH` directly.
- **`PyControl.py`** — the control-law abstraction (`PyControl` class). Selects a backend from the extension of
  `control_file`: `.py` → pure Python, `.m` → MATLAB (via `matlab.engine`), `.slx` → Simulink (stepped through
  the MATLAB engine's `sim()`), `.fmu` → compiled FMI 2.0 co-simulation FMU (via `fmpy`, wrapped in
  `SimulinkFMUController`). All backends are driven every timestep through the same entry point,
  `PyControl_DoControllerStep(instantaneous_state, Dt)`, returning a `{signal_name: value}` dict (`F1..F20`
  control surfaces, `E1..E20` engine/aux signals).
  - Every backend's user-supplied code must define a class named `UserController` with `__init__` (optionally
    taking a precomputed-data file path) and `step(instantaneous_state, Dt) -> dict`.
  - MATLAB engine startup flags are chosen per-method (no JVM for pure `.m`, JVM but no desktop for headless
    Simulink, full desktop if `show_simulink=True`).
  - For `.slx`, WingLoop forces a fixed-step discrete solver and overrides `SampleTime` on every block to match
    WingLoop's own `Dt` — the `.slx` file's own settings are not authoritative at runtime. See
    "Rules for writing a Simulink controller" below.
  - For `.fmu` with `rebuild_fmu=True`, the FMU is re-exported from the matching `.slx` before each run so
    `UserController.m`'s `assignin('base', ...)` values get baked into the FMU as new initial parameters; with
    `rebuild_fmu=False` the FMU runs standalone (no MATLAB engine needed) using whatever was baked in previously.
- **`PyControl_IO.py`** — the data interface layer between ASWING's text files and Python. Central data
  structure is the `data_dict` (`ModelName`, `ModelStates`, `ModelVariables: {name: {values, unit, latex}}`).
  Key functions: `initialize_data_dict`, `read_aswing_file` (parses ASWING's loosely-structured text output
  using regexes built dynamically from the requested variable names — variable names can contain spaces, e.g.
  `"earth X"`), `python2text` (writes ASWING-compatible control input files), `export_data_dict`/
  `import_data_dict` (JSON round-trip, optionally compressed).
- **`PyControl_Plot.py`** — `ASWINGLivePlotter`: real-time/post-run plotting of the `data_dict` time series
  (matplotlib), including convergence-loss overlays.
- **`PyControl_additional.py`** — legacy utilities from earlier WingLoop versions (e.g. an older standalone
  `PIDController`/`UAV_control_Strategy`); not part of the current call path from `WingLoop.py`.
- **`ASW_Helpers.py`** — post-processing utilities unrelated to the control loop itself: generating a
  PostScript plot file from ASWING, then producing analysis videos (`Generate_Analysis_Videos`) and
  stroboscopic images (`Generate_Strobe_Plot`) from it, and general timeseries plotting.

### Data flow per timestep (`WingLoop.Performing_K_iterations_ASWING`)

1. Tell ASWING to write its state (`W` command) to `output`, overwriting the previous content.
2. `read_aswing_file` parses `output` into `self.WingLoop_LogFile` (the `data_dict`), appending to history.
3. If plotting is enabled, the live plot is updated.
4. The latest state vector is passed to `PyControl.PyControl_DoControllerStep`.
5. The returned control dict is serialized via `python2text` into `input`.
6. ASWING is told to run `K` more iterations (`x input` / `x`) using that `input` file.

`self.count` tracks iterations already executed in ASWING and determines whether the next `x` command is
relative (`-K`) or absolute — getting this wrong desyncs Python's iteration counter from ASWING's actual state.

### Rules for writing a Simulink controller (`.slx`)

(from the repo README — required for a `.slx` controller to work correctly under WingLoop)

- Solver: Fixed-step, "discrete (no continuous states)". Leave step size `-1` (WingLoop overrides it with `Dt`
  at runtime). Continuous blocks are not supported.
- To get the current timestep inside the model: Clock → Unit Delay, subtract Unit Delay output from Clock
  output.
- Derivative filter coefficient `N` (e.g. on a Discrete PID): must satisfy `N·Dt <= 1` (filter pole at
  `1 - N·Dt`); `N·Dt > 1` is unstable in Simulink even when equivalent Python/MATLAB controllers are stable.
  Use the value WingLoop pushes to the base workspace as `Nfilter = 1/Dt` (i.e. `N·Dt = 1.0`) rather than
  hardcoding `N`.
- Inputs: single `Inport` named `statein` receiving the full WingLoop state vector.
- Outputs: for each control signal, both an `Outport` **and** a `To Workspace` block with the same
  ASWING-compatible name (e.g. `F1`, `E1`) — the Outport is read by WingLoop, the To Workspace block enables
  logging/inspection.
- Workspace variables (gains, lookup tables, etc.) must be pushed via `assignin('base', ...)` inside
  `UserController.m`'s constructor, not stored inside the `.slx` itself — WingLoop guarantees `UserController.m`
  runs before the first `sim()` call.

### Directory map

- `wingloop_testrun/` — end-to-end example/validation case (one aircraft, PID controller implemented five ways:
  Python, MATLAB, Simulink, Simulink FMU, ASWING-native). `aswing_geometry/` holds the ASWING-side files
  (`.asw`, `.pnt`, `.set`, `.state`, `.gust`, and the `input`/`output` exchange files). Per-backend controller
  folders: `python_controller/`, `matlab_controller/`, `simulink_controller/`, `simulink_controller_2/`.
- `test_files/` — smaller fixtures (test aircraft + test controllers per backend) used for ad hoc/manual
  testing of individual modules, not a pytest suite.
- `icon/` — GUI icon asset.

### Performance notes (from README benchmarks)

Simulink (non-FMU) is dramatically slower than the other backends (~9–13x baseline vs ~1.1–1.5x for
Python/MATLAB/FMU) because each step launches through the full MATLAB/Simulink `sim()` machinery. Prefer Python
or MATLAB for simple controllers; use non-FMU Simulink only to validate model behavior, then switch to a
compiled FMU (`rebuild_fmu=True` once, then `False`) for speed.