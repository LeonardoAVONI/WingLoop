# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

WingLoop is a Python framework that gives [ASWING](https://web.mit.edu/drela/Public/web/aswing/) (a Fortran
structural/aerodynamic aircraft analysis tool) closed-loop control capability. ASWING natively only supports
linear bi-scheduled controllers; WingLoop drives ASWING as a subprocess, exchanging state/control data through
text files each timestep, so that arbitrary control laws — written in Python, MATLAB, or Simulink — can be
applied during a time-transient simulation. Version 3.0 lives on the `v3` branch (`main` is V2).

This directory (`WingLoop_Library/`) is the installable package, one level below the repo root (which holds
`pyproject.toml`, `README.md`, `LICENCE`, `CONTRIBUTING.md`, `tests/`). The package is imported as
`WingLoop_Library` (distribution name `WingLoop`).

Reference publication: Avoni, Bronz, Condomines, Moschetta, "Enhancing ASWING Flight Dynamics Simulations with
Closed-Loop Control for Flexible Aircraft," AIAA 2025-3425, 2025. Simulink V2 method: Matteo Alfonso Incarbone.

## License / contribution model

CC BY-NC-SA 4.0 (non-commercial). Contributions require agreeing to the CLA in `CONTRIBUTING.md` (record it in
`CONTRIBUTORS.md`). Keep the copyright/license header block at the top of every source file intact.

## Setup and running

```bash
pip install -e .                 # from the repo root; numpy + matplotlib only
pip install -e .[fmu,test]       # FMPy (.fmu backend), pytest
pip install "matlabengine==25.2.*"   # MATLAB backends: version must match the MATLAB release (R2025b here)
```

`matlab.engine` and `fmpy` are imported lazily (only by the backends that use them); `import WingLoop_Library`
must keep working without them and without importing matplotlib (`tests/test_wingloop.py` checks this).
On this machine the conda env `Matlab` (`~/miniconda3/envs/Matlab/bin/python`) has every optional dependency;
MATLAB R2025b is at `/usr/local/MATLAB/R2025b`. ASWING aliases in `~/.bashrc`: `aswing_stable`,
`aswing_modified` (binaries `~/Desktop/02_ASWING/ASWING_{Stable,Modified}/bin/aswing`).

Tests:

```bash
python -m pytest                                   # repo root; no ASWING needed (tests/fake_aswing.py)
python -m pytest tests/test_wingloop.py -k retry   # one test
cd WingLoop_Library/wingloop_testrun
python wingloop_testrun.py py                      # one real run (py|mat|sim|fmu|fmu_rebuild|simv2|fixed|native)
python run_all_backends.py                         # all backends + cross-check report (results_v3/report.md)
```

`run_all_backends.py` is the regression check for anything touching the loop, the I/O or a backend: every
backend implements the same PID, so Python, MATLAB, Simulink V1, FMU and Simulink V2 must agree to round-off
(currently bit-identical) and the report flags a one-step lag. It needs ASWING (alias `aswing_stable`) and
MATLAB/Simulink with Simulink Coder (FMU export).

Do not build ASWING yourself (its own repos say so); running it is fine.

## Architecture

Time-marching loop split across two processes: ASWING (subprocess) and Python. Per chunk of K steps:
controller(state at t_k) → `input` file → ASWING marches K steps (`x input`, `Dt -K`) → `W output` →
`parse_point_file` → record. The initial point (t = 0) is read before the first chunk, so the first chunk is
controlled too and the record holds t = K·Dt … N·Dt (plus t = 0 with `record_initial=True`).

- **`Aswing_Director.py`** — subprocess driver. **Prompt synchronisation**: ASWING runs with
  `GFORTRAN_UNBUFFERED_PRECONNECTED=y`; a reader thread collects raw stdout; `send_command_and_receive` returns
  when the output ends with a menu prompt (`PROMPT_RE`: `   c>  `, `s>`, `i>`, `r>`, ` y/n>` — printed *without*
  newline) or matches `expect=`. Questions printed as plain lines are not prompts: callers must pass `expect`
  (e.g. `MARCH_QUESTION_RE` after `x`, the overwrite question of file writes); without `expect` a command
  returns after `quiet_time` of silence (fallback for arbitrary user commands). A file written by ASWING is
  closed before the next prompt, so `write_point_file` needs no polling (it still checks
  `ENDEDPRINTINGSTATES`). Every command has a timeout (`write_timeout`, default 1200 s); on timeout the process
  group is killed and `ASWINGHangError` raised (`ASWINGCrashError` on exit). `sync="timer"` keeps V2's
  fixed-wait behaviour (used by the interactive video helper in `ASW_Helpers.py`). `stdout_queue` is a bounded
  mirror kept for callers that read it (the 20_ project's `set_aero_flags`).
- **`WingLoop.py`** — orchestrator. `Time_Transient_Simulation` owns the loop: unsteady OPER entry (checks the
  `.OPERu` prompt; toggles back if `.` turned it steady), optional `Set_Aero_Flags`, the chunk loop, optional
  retry (`retry_substeps`, off by default: ASWING_Modified sub-steps failed steps itself, TSPLIT), stop
  conditions, atmosphere update, hang/crash → `sim_status`. **Record each point exactly once**: parse into a
  dict, decide (retry), then `_commit` — never append and pop (the V2 bug class: lost/duplicated samples).
  Channels added per step: `IsConverged`, `NewtonRestarts`/`TSplits` (counted in the march output), `Retried`,
  `Time_ASWING` (`Time` is k·Dt in float64), `cmd_<name>` (everything the controller returned, including
  non-ASWING flags, which are not sent to ASWING), `alpha_ref`/`beta_ref` (from the state block, only when
  `KPTOT >= 120`, i.e. ASWING_Modified: index 18·IITOT + 12·NJOIN + NNTOT + kp − 1). Open-loop controllers
  (`schedule(times)`, optional `takeover(channels)`) write one time-history file; ASWING evaluates traces at
  the absolute end time of each step, so one row (t_{k+1}, u_k) per step is exact. `max_time_points = 9997`
  (ASWING `NPNTX = 10001` minus TSPLIT scratch slots).
- **`PyControl.py`** — backends selected by extension: `.py`, `.m`, `.slx` (+ `simulink_mode="v2"`), `.fmu`.
  Every backend gets the full state vector and returns `{name: value}`; user code is a `UserController` class
  with `step(state, Dt)` (Python may take a third `channels` argument: named ASWING values). Before the first
  step WingLoop pushes `Nfilter = 1/Dt` and **`WL_NumStates`** (state length) to the MATLAB base workspace;
  Simulink models size `statein` with `WL_NumStates`. FMU re-export (`rebuild_fmu=True`) runs at the first
  step (it needs `WL_NumStates`); its time is reported as start-up (`fmu_rebuild_time`).
- **`SimulinkV2.py` + `simulink_v2/`** — Simulink V2: one `sim()` for the whole run. `wingloop_v2_make_model.m`
  copies the user's V1-style model to `<model>_wlv2.slx`, replacing `statein` with the `AswingPlant` MATLAB
  System block and the command Outports with a Mux into it. `AswingPlant` has no direct feedthrough
  (`outputImpl` = y_k from WingLoop, `updateImpl(u_k)` sends u_k), which gives u_k = C(y_k) like every other
  backend and no algebraic loop. Binary TCP protocol (little-endian) documented in `SimulinkV2.py`; WingLoop is
  the server. Launch: handshake JSON → `wingloop_v2_run.m` (engine, background) on Linux;
  `wingloop_v2_windows_launcher.m` for MATLAB on Windows + WingLoop in WSL (untested). Start-up (wrapper build +
  compile, ~30 s) happens at the first step and is reported separately (`startup_time`).
- **`PyControl_IO.py`** — `parse_point_file` (pure; raises `IncompletePointFile`; header regex stops at the
  spanwise beam tables; first match per variable = the Peng fix), `read_aswing_file` (V2 interface on top),
  `export_data_dict`/`import_data_dict` (`.npz` default, ~3x smaller than `.json.gz`: states float32,
  which keeps ASWING's single-precision values exactly; `.json.gz` still read/written), `python2text`, `python2text_schedule`.
- **`PyControl_Plot.py`** — live/post-run plotting; imported only by `InitializePlot` with plot variables.
- **`ASW_Helpers.py`** — videos / strobe images / timeseries plots (merged V2 + M. A. Incarbone's video
  options). Loaded lazily through `WingLoop_Library.__getattr__`.
- **`PyControl_additional.py`** — legacy, not on the call path.

### Rules for a Simulink controller (V1, FMU and V2 alike)

- Fixed-step discrete solver, discrete blocks only (WingLoop forces the step and every positive sample time to
  Dt). Clock → Unit Delay difference gives Dt inside the model.
- Derivative filter: use `Nfilter` (= 1/Dt, N·Dt = 1); N·Dt > 1 is unstable.
- One root Inport `statein`, dimension `WL_NumStates` (anything that indexes it, e.g. a Selector, uses
  `WL_NumStates` as input width). One root Outport per command named `F1..F20`/`E1..E20` (+ To Workspace for
  logging in V1).
- Workspace variables are pushed by `UserController.m` (`assignin('base', ...)`), which runs before `sim`.

### Compatibility with V2 callers

The 20_Flexible_Models_and_Control project calls `Launch_WingLoop_Control`, `InitializePlot`, `Launch_ASWING`
(with `aswing_fullpath` = folder), `ASW_handler.write_timeout`, `ASW_handler.send_command_and_receive`,
`ASW_handler.send_writefile_command_and_receive`, `ASW_handler.stdout_queue`, `Time_Transient_Simulation(Dt,N,K)`,
`Outputting_The_Results`, `Closing_WingLoop`, `import_data_dict`. Keep these working. V3 behaviour changes
they see: the first chunk is controlled and recorded (time grid K·Dt…N·Dt), `.npz` output by default
(`Outputting_The_Results(fmt="json")` for `.json.gz`), retry off by default.

### Repo hygiene

`__pycache__/` and `slprj/` artifacts are tracked from V2 days; don't stage new copies. Run artefacts are
ignored (`results_v3/`, `*_wlv2.slx`, handshake files). `ideas_from_20_folder.md` (repo root) is the
requirements list V3 implements (items marked NOT_THIS excluded).
