# =============================================================================
# WingLoop
# =============================================================================
# Copyright (c) 2024-2026 Leonardo Avoni (avonileonardo@gmail.com)
#
# This file is part of WingLoop.
# WingLoop is licensed under CC BY-NC-SA 4.0 (Non-Commercial use only).
# Full license: https://creativecommons.org/licenses/by-nc-sa/4.0/
#
# For commercial use, contact the author: avonileonardo@gmail.com
#
# If you use WingLoop in academic work, please cite:
#   Avoni et al., "Enhancing ASWING Flight Dynamics Simulations with
#   Closed-Loop Control for Flexible Aircraft," AIAA 2025-3425.
#   https://arc.aiaa.org/doi/10.2514/6.2025-3425
# =============================================================================
"""
====================================================================================
WingLoop - Aeroservoelastic Control Interface for ASWING
Version 3.0
====================================================================================

Author: Leonardo Avoni (avonileonardo@gmail.com)
Simulink V2 method: Matteo Alfonso Incarbone (lpmatteo241@gmail.com)
Initial release: 25 Oct 2024. Version 3.0: Nov 2026.

WingLoop marches an ASWING time-transient simulation step by step (or in chunks of K
steps). After each chunk it reads the ASWING state, passes it to a control law
(Python, MATLAB, Simulink V1, Simulink FMU or Simulink V2, see PyControl.py) and
writes the commands ASWING applies over the next chunk.

Typical use (see wingloop_testrun/wingloop_testrun.py):

    WL = WingLoop()
    WL.Launch_WingLoop_Control(cntrl_directory, cntrl_filename, timestep=Dt)   # or None, None: fixed stick
    WL.Launch_ASWING(aswing_fullpath=..., sim_directory=..., asw_filename=...)
    WL.Deactivate_Graphics(); WL.Load_Files("case.pnt"); ...; WL.Load_Files("case.state")
    converged = WL.Time_Transient_Simulation(Dt, N, K)
    WL.Outputting_The_Results("run")      # run.npz (default) or run.json.gz (fmt="json")
    WL.Closing_WingLoop()

Time grid and timing (V3)
    The initial (trimmed) point is read at t = 0 and every chunk end is recorded, so
    the recorded times are K*Dt, 2*K*Dt, ..., N*Dt (plus t = 0 with record_initial=True).
    The command applied over [t_k, t_k + K*Dt] is computed from the state at t_k,
    starting at t_0 (V2 marched the first chunk with the trim controls, unrecorded).
    "Time" holds k*Dt in float64; ASWING's printed time (4 decimals) is "Time_ASWING".

Per-step channels added by WingLoop
    IsConverged, NewtonRestarts, TSplits (counted in ASWING's output), Retried
    (0: no retry, 1: retried and converged, 2: retried and still not converged),
    Time_ASWING, cmd_<name> (every value the controller returned for the step,
    including user flags that are not ASWING commands), alpha_ref / beta_ref
    (ASWING_Modified only, from the "additional variables" of the state block).

Robustness
    ASWING is driven with prompt synchronisation (Aswing_Director.py): every command
    returns when ASWING is done, every point file is checked for ENDEDPRINTINGSTATES,
    every command has a timeout. A hang or crash ends the run with
    self.sim_status = "hung at step k: ..." / "crashed at step k: ..." and
    Time_Transient_Simulation returns False (the data recorded so far is kept).
====================================================================================
"""

import os
import re
import sys
import json
import time
import socket
import hashlib
import platform
import subprocess
import warnings

import numpy as np

from .Aswing_Director import Aswing_Director, ASWINGHangError, ASWINGCrashError, MARCH_QUESTION_RE
from .PyControl import PyControl
from .PyControl_IO import (initialize_data_dict, _build_pattern, parse_point_file, python2text,
                           python2text_schedule, export_data_dict, model_states_array, _store_state)

__version__ = "3.0.0"

# ASWING truncates file names to 80 characters (CHARACTER*80, e.g. VGPGET)
MAX_ASWING_FILENAME = 80

GLOBAL_VARIABLES = [
    # Position & attitude
    "Time", "earth X", "earth Y", "earth Z", "Heading", "Elev.", "Bank",
    "Alpha", "Beta", "Velocity",
    # Angular velocities / accelerations
    "Wx", "Wy", "Wz", "Wdotx", "Wdoty", "Wdotz",
    # Linear velocities / accelerations
    "Ux", "Uy", "Uz", "Udotx", "Udoty", "Udotz",
    # Moments, forces
    "sum Mx", "sum My", "sum Mz", "sum Fx", "sum Fy", "sum Fz",
    # Aero & reference quantities
    "Lift", "Density", "Ref.Area", "Weight", "Dyn.Pr.", "Ref.Span",
    "Load Fac", "VIAS", "Ref.Chrd", "Mach", "VTAS", "MachPG",
    # Aero coefficients
    "CL", "CD", "L/D", "Cl'", "Cm", "CDi", "e", "Cn'",
    # Convergence (from the Status line)
    "IsConverged",
    "Op.Point", "altitude",
]
CONTROL_ELEMENTS = [f"F{i}" for i in range(1, 21)] + [f"E{i}" for i in range(1, 21)]
WINGLOOP_CHANNELS = ["Time_ASWING", "NewtonRestarts", "TSplits", "Retried", "alpha_ref", "beta_ref"]
_ASWING_COMMAND_RE = re.compile(r"^[FE]([1-9]|1[0-9]|20)$")
_RESTART_RE = re.compile(r"Newton restart #")
_TSPLIT_RE = re.compile(r"TSPLIT: .*re-solving")


def _check_filename(name, what="file"):
    if name is not None and len(str(name)) > MAX_ASWING_FILENAME:
        raise ValueError(f"{what} name has {len(str(name))} characters; ASWING truncates names to "
                         f"{MAX_ASWING_FILENAME}: {name!r}. Use a shorter or relative name.")


def _git_commit():
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        out = subprocess.run(["git", "-C", here, "rev-parse", "HEAD"], capture_output=True, text=True, timeout=5)
        dirty = subprocess.run(["git", "-C", here, "status", "--porcelain", "--untracked-files=no"],
                               capture_output=True, text=True, timeout=5)
        if out.returncode == 0:
            return out.stdout.strip() + ("-dirty" if dirty.stdout.strip() else "")
    except Exception:
        pass
    return None


def file_sha256(path, cache=True):
    """SHA-256 of a file, cached in ~/.cache/wingloop/hashes.json by (path, size, mtime):
    the ASWING binaries are ~1.7 GB."""
    st = os.stat(path)
    key = f"{os.path.abspath(path)}|{st.st_size}|{int(st.st_mtime)}"
    cache_file = os.path.join(os.path.expanduser("~"), ".cache", "wingloop", "hashes.json")
    table = {}
    if cache:
        try:
            with open(cache_file) as f:
                table = json.load(f)
            if key in table:
                return table[key]
        except (OSError, ValueError):
            table = {}
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 22), b""):
            h.update(block)
    digest = h.hexdigest()
    if cache:
        try:
            os.makedirs(os.path.dirname(cache_file), exist_ok=True)
            table[key] = digest
            tmp = f"{cache_file}.{os.getpid()}.tmp"
            with open(tmp, "w") as f:
                json.dump(table, f)
            os.replace(tmp, cache_file)
        except OSError:
            pass
    return digest


class WingLoop:
    """Closed-loop time marching of an ASWING aircraft. See the module docstring."""

    # ASWING stores at most NPNTX = 10001 points; a march stops at NPNTX - 3 (TSPLIT scratch slots)
    max_time_points = 9997

    def __init__(self):
        # exchange files (per instance, so parallel workers can share a folder)
        self.input_filename = "input"
        self.output_filename = "output"
        self.fixed_stick = True
        self.PyControl = None
        self.ASW_handler = None
        # plotting / recording
        self.LivePlot = False
        self.WingLoop_IsPlotting = False
        self.WingLoop_Liveplot = None
        self.WingLoop_LogFile = None
        self._recording_N = None
        self.writingtime = 0.0
        # atmosphere update (M. A. Incarbone)
        self.update_aswing_atmosphere = False
        self.initial_altitude_km = None
        self.atmosphere_gravity = 9.80665
        # alpha_ref / beta_ref: PARAM index -> channel (ASWING_Modified, INDEXP.INC KPAREF/KPBREF)
        self.param_channels = {"alpha_ref": 70, "beta_ref": 71}
        self.sim_status = "not run"
        self.metadata = {}
        self.count = 0

    # ================================================================== setup
    def Configure_Atmosphere_Update(self, enabled=False, initial_altitude_km=None, gravity=9.80665):
        """Update ASWING's atmosphere from the current earth-Z before each chunk (M. A. Incarbone).

        Altitude [km] = initial_altitude_km + earth Z [m] / 1000, sent with the OPER "A"
        command (density, speed of sound, viscosity). Needs ASWING altitude units in km.
        """
        self.update_aswing_atmosphere = bool(enabled)
        self.initial_altitude_km = None if initial_altitude_km is None else float(initial_altitude_km)
        self.atmosphere_gravity = float(gravity)
        if self.update_aswing_atmosphere:
            if self.initial_altitude_km is None or not np.isfinite(self.initial_altitude_km):
                raise ValueError("initial_altitude_km must be finite when atmosphere updating is enabled.")
            if not np.isfinite(self.atmosphere_gravity):
                raise ValueError("Atmosphere gravity must be finite.")

    def Launch_ASWING(self, aswing_fullpath=None, aswing_alias=None, sim_directory=".", asw_filename=None,
                      print_output=False,
                      timer_text=0.03,
                      finished_writing_check_timestep=0.002,
                      input_filename="input",
                      output_filename="output",
                      write_timeout=1200.0,
                      command_timeout=None,
                      sync="prompt"):
        """
        Start ASWING in `sim_directory` (WingLoop and ASWING then work in that folder).

        aswing_fullpath : ASWING executable or its folder (preferred). If None, aswing_alias
            is resolved through bash (slow; do it once in the main process for parallel runs).
        sim_directory : folder with the case files; relative exchange files live there.
        asw_filename : geometry (.asw, relative to sim_directory or absolute).
        print_output : echo ASWING's output.
        timer_text : V2 fixed wait per command, used only with sync="timer".
        finished_writing_check_timestep : polling interval of the fallback file checks [s].
        input_filename, output_filename : per-instance exchange files (parallel workers).
        write_timeout : timeout of a march or file write [s] (default 1200: hard steps can
            take minutes under load). command_timeout : other commands (default write_timeout).
        sync : "prompt" (V3) or "timer" (V2 behaviour).
        """
        self.sim_directory = os.path.abspath(sim_directory)
        if not os.path.isdir(self.sim_directory):
            raise FileNotFoundError(f"ASWING simulation directory not found: {self.sim_directory}")
        if asw_filename:
            check = asw_filename if os.path.isabs(asw_filename) else os.path.join(self.sim_directory, asw_filename)
            if not check.endswith(".asw"):
                check += ".asw"
            if not os.path.isfile(check):
                raise FileNotFoundError(f"ASWING geometry file not found: {check}")
            _check_filename(asw_filename, "Geometry file")
        for n in (input_filename, output_filename):
            _check_filename(n, "Exchange file")
        self.print_output = print_output
        self.count = 0
        self.input_filename = input_filename
        self.output_filename = output_filename
        self.initial_path = os.getcwd()
        self.ASW_handler = Aswing_Director(aswing_path=aswing_fullpath, aswing_alias=aswing_alias,
                                           wait_time=timer_text,
                                           finished_writing_file_check_timestep=finished_writing_check_timestep,
                                           write_timeout=write_timeout, command_timeout=command_timeout,
                                           sync=sync)
        self.ASW_handler.start_aswing(directory=self.sim_directory, filename=asw_filename,
                                      print_output=self.print_output)
        self.metadata.update({"aswing_executable": self.ASW_handler.aswing_exe,
                              "sim_directory": self.sim_directory, "geometry": asw_filename,
                              "sync": sync})
        try:      # cached per (path, size, mtime): only the first run with a new binary pays for it
            self.metadata["aswing_sha256"] = file_sha256(self.ASW_handler.aswing_exe)
        except OSError:
            self.metadata["aswing_sha256"] = None

    def Launch_WingLoop_Control(self, cntrl_directory, cntrl_filename, timestep,
                                precomputed_filename=None,
                                rebuild_fmu_file=False,
                                show_simulink_window=False,
                                simulink_mode="v1",
                                simulink_launch="engine",
                                simulink_port=0,
                                simulink_handshake_file=None):
        """
        Set up the control law (cntrl_directory / cntrl_filename None: fixed stick).

        The file extension selects the backend: .py Python, .m MATLAB, .slx Simulink V1
        (simulink_mode="v1") or Simulink V2 (simulink_mode="v2"), .fmu Simulink FMU.
        Simulink V2: simulink_launch="engine" (Linux, MATLAB started by WingLoop) or
        "external" (Windows + WSL, MATLAB started by the user, untested); simulink_port 0
        picks a free port; simulink_handshake_file is where the run parameters are written.
        """
        self.control_timestep = timestep
        if (cntrl_directory is not None) and (cntrl_filename is not None):
            self.PyControl = PyControl(
                control_directory=cntrl_directory,
                control_file=cntrl_filename,
                precomputed_file=precomputed_filename,
                Dt=timestep,
                rebuild_fmu=rebuild_fmu_file,
                show_simulink=show_simulink_window,
                simulink_mode=simulink_mode,
                simulink_launch=simulink_launch,
                simulink_port=simulink_port,
                simulink_handshake_file=simulink_handshake_file,
            )
            self.fixed_stick = False
            self.metadata.update({"controller": os.path.join(cntrl_directory, cntrl_filename),
                                  "controller_method": self.PyControl.method})
        else:
            self.PyControl = None
            self.fixed_stick = True
            self.metadata.update({"controller": None, "controller_method": "fixed_stick"})

    def Configure_Recording(self, variables=None, N_steps=None):
        """
        Set up the recorded data (WingLoop_LogFile). Called by InitializePlot, or
        automatically by Time_Transient_Simulation; call it yourself to record a
        different list of ASWING global variables (default GLOBAL_VARIABLES).
        """
        requested = list(variables) if variables is not None else list(GLOBAL_VARIABLES)
        if "IsConverged" not in requested:
            requested.append("IsConverged")
        if "Time" not in requested:
            requested.insert(0, "Time")
        self.rename_map = {}
        for ctrl in CONTROL_ELEMENTS:
            raw = ("Flap " if ctrl[0] == "F" else "Peng ") + ctrl[1:]
            requested.append(raw)
            self.rename_map[raw] = ctrl
        self.WingLoop_LogFile = initialize_data_dict(requested, self.rename_map, {}, N_steps=N_steps)
        self._compiled_pattern = _build_pattern(self.WingLoop_LogFile, self.rename_map)
        for ch in WINGLOOP_CHANNELS:
            self.WingLoop_LogFile["ModelVariables"][ch] = {"values": [], "unit": None, "latex": None}
        self.WingLoop_LogFile["ModelVariables"]["Time_ASWING"]["unit"] = "s"
        self._recording_N = N_steps

    def InitializePlot(self, liveplot, plot_variables, plot_sim_time, plot_refreshtime,
                       plot_size=(16, 10), N_steps=None):
        """
        Set up recording and (optionally) plotting. Plotting is off unless plot_variables
        is given; liveplot=True updates the figure during the run, otherwise it is drawn at
        the end (and saved as a .pdf by Outputting_The_Results). matplotlib is imported
        only when plotting is requested.
        """
        self.Configure_Recording(N_steps=None if N_steps is None else N_steps + 1)
        self.LivePlot = bool(liveplot)
        self.WingLoop_IsPlotting = all(v is not None for v in (plot_variables, plot_sim_time,
                                                                plot_refreshtime, plot_size))
        if self.WingLoop_IsPlotting:
            from .PyControl_Plot import ASWINGLivePlotter
            self.WingLoop_Liveplot = ASWINGLivePlotter(parameter_list=plot_variables,
                                                       total_sim_time=plot_sim_time,
                                                       refresh_interval=plot_refreshtime,
                                                       figsize=plot_size)
        self.writingtime = 0.0

    def Load_Files(self, filename):
        """Main menu: load a file by extension (.asw, .pnt, .set, .state, .gust, .con)."""
        _check_filename(filename)
        self.ASW_handler.Load_Files_ASW(filename)

    def Deactivate_Graphics(self):
        """Turn ASWING's graphics off (main menu: PLPA, G, return)."""
        self.ASW_handler.send_command_and_receive("PLPA")
        self.ASW_handler.send_command_and_receive("G")
        self.ASW_handler.send_command_and_receive("")

    _FLAG_RE = {"freeze_vl": re.compile(r"F\s?reeze VL matrices.*?:\s*([TF])"),
                "lag": re.compile(r"L\s?ag aero effects modeled\s*:\s*([TF])")}

    def _read_flags(self, text):
        out = {}
        for key, rx in self._FLAG_RE.items():
            hits = rx.findall(text)
            if hits:
                out[key] = hits[-1] == "T"
        return out

    def Set_Aero_Flags(self, freeze_vl=None, lag=None, newton_iters=None):
        """
        OPER menu: set and read back the k-menu settings "Freeze VL matrices" and "Lag aero
        effects" (the lag setting is not stored in .set files) and optionally the Newton
        iteration limit. Raises RuntimeError if the flags read back differ. Call at the
        OPER prompt (Time_Transient_Simulation(aero_flags=...) does it for you).
        Returns {"freeze_vl": bool, "lag": bool} (the flags ASWING printed).
        """
        cmd = self.ASW_handler.send_command_and_receive
        text, _ = cmd("k")
        if newton_iters is not None:
            text, _ = cmd(f"I {int(newton_iters)}")
        flags = self._read_flags(text)
        for key, want, letter in (("freeze_vl", freeze_vl, "F"), ("lag", lag, "L")):
            if want is None:
                continue
            if key not in flags:
                cmd("")
                raise RuntimeError(f"ASWING k menu shows no {key!r} setting (this ASWING build may not have it)")
            if flags[key] != bool(want):
                text, _ = cmd(letter)
                flags.update(self._read_flags(text))
        cmd("")
        for key, want in (("freeze_vl", freeze_vl), ("lag", lag)):
            if want is not None and flags.get(key) != bool(want):
                raise RuntimeError(f"ASWING k menu flags are {flags}, wanted freeze_vl={freeze_vl}, lag={lag}")
        self.metadata["aero_flags"] = flags
        return flags

    # ================================================================== time march
    def _read_point(self):
        t0 = time.time()
        self.ASW_handler.write_point_file(self.output_filename)
        p = parse_point_file(self.output_filename, self._compiled_pattern, self.rename_map,
                             wanted=self.WingLoop_LogFile["ModelVariables"])
        self.writingtime += time.time() - t0
        return p

    def _commit(self, p, t, restarts=0, tsplits=0, retried=0, commands=None):
        """Append one point (parsed by parse_point_file) to the record, exactly once."""
        log = self.WingLoop_LogFile
        V = log["ModelVariables"]
        log["ModelName"] = p["name"]
        log["ModelParameters"].update(p["params"])
        for name, value in p["values"].items():
            if name in V:
                V[name]["values"].append(value)
                if V[name]["unit"] is None and p["units"].get(name) is not None:
                    V[name]["unit"] = p["units"][name]
        if "Time" in p["values"]:
            V["Time"]["values"][-1] = float(t)       # full precision, k*Dt
        else:
            V["Time"]["values"].append(float(t))
        V["Time_ASWING"]["values"].append(p["values"].get("Time", np.nan))
        V["IsConverged"]["values"].append(bool(p["converged"]))
        V["NewtonRestarts"]["values"].append(int(restarts))
        V["TSplits"]["values"].append(int(tsplits))
        V["Retried"]["values"].append(int(retried))
        S = p["states"]
        prm = log["ModelParameters"]
        kptot = prm.get("KPTOT")
        for ch, kp in self.param_channels.items():
            val = np.nan
            if S is not None and kptot is not None and kptot >= 120 and None not in (
                    prm.get("IITOT"), prm.get("NJOIN"), prm.get("NNTOT")):
                idx = 18 * prm["IITOT"] + 12 * prm["NJOIN"] + prm["NNTOT"] + kp - 1
                if idx < len(S):
                    val = float(S[idx])
            V[ch]["values"].append(val)
        if commands:
            for name, value in commands.items():
                key = "cmd_" + name
                if key not in V:
                    V[key] = {"values": [np.nan] * (len(V["Time"]["values"]) - 1), "unit": None, "latex": None}
                V[key]["values"].append(float(value) if value is not None else np.nan)
        for key in V:
            if key.startswith("cmd_") and len(V[key]["values"]) < len(V["Time"]["values"]):
                V[key]["values"].append(np.nan)
        if S is not None:
            _store_state(log, S, True)

    def _channels(self, p, t):
        ch = dict(p["values"])
        ch["Time"] = float(t)
        ch["IsConverged"] = bool(p["converged"])
        return ch

    def _enter_unsteady_oper(self, aero_flags):
        cmd = self.ASW_handler.send_command_and_receive
        out, _ = cmd("oper")
        if aero_flags:
            self.Set_Aero_Flags(**aero_flags)
        out, _ = cmd(".")
        if self.ASW_handler.sync == "prompt" and ".OPERs" in out[-200:]:
            out, _ = cmd(".")            # it was already unsteady: toggle back
        if self.ASW_handler.sync == "prompt" and ".OPERu" not in out[-200:]:
            raise RuntimeError(f"Could not switch OPER to unsteady mode. Output:\n{out[-600:]}")

    def _update_atmosphere(self, p):
        if not self.update_aswing_atmosphere:
            return
        z = p["values"].get("earth Z")
        unit = str(p["units"].get("altitude") or self.WingLoop_LogFile["ModelVariables"]
                   .get("altitude", {}).get("unit") or "km").strip().lower()
        if z is None:
            raise RuntimeError("ASWING output does not contain earth Z (needed for the atmosphere update).")
        if unit not in ("km", "kilometer", "kilometers"):
            raise RuntimeError(f"Dynamic atmosphere update expects altitude in km, the output reports {unit!r}.")
        alt = self.initial_altitude_km + float(z) * 1.0e-3
        if not np.isfinite(alt):
            raise RuntimeError(f"Computed ASWING altitude is not finite: {alt}.")
        self.ASW_handler.send_command_and_receive(f"A {alt:.9g} {self.atmosphere_gravity:.9g}")

    def _controller_commands(self, p, t, Dt_chunk, takeover_state):
        """Commands for the next chunk: {name: value} (all) and the ASWING subset."""
        ctrl = self.PyControl
        user = getattr(ctrl, "python_controller_instance", None)
        if takeover_state["open_loop"]:
            if hasattr(user, "takeover") and user.takeover(self._channels(p, t)):
                takeover_state["open_loop"] = False
                takeover_state["takeover_time"] = t
                print(f"[WingLoop] t = {t:g} s: controller takes over (closed loop from here)")
            else:
                return None, None
        out = ctrl.PyControl_DoControllerStep(instantaneous_state=p["states"], Dt=Dt_chunk,
                                              channels=self._channels(p, t))
        aswing = {k: v for k, v in out.items() if _ASWING_COMMAND_RE.match(k)}
        return out, aswing

    def Time_Transient_Simulation(self, Dt, N, K=1, stop_if_notconverged=True, stop_callback=None,
                                  stop_condition=None, retry_substeps=0, record_initial=False,
                                  aero_flags=None):
        """
        March N time steps of Dt from the loaded (trimmed) state, controller called every K steps.

        Parameters
        ----------
        Dt, N, K : time step [s], number of steps, steps per controller call (one record per chunk).
        stop_if_notconverged : stop at the first step that did not converge (after the retry, if any).
        stop_callback : f(wingloop) -> bool, called after every chunk; True stops the run.
        stop_condition : f(channels) -> bool on the latest named channels ({"Alpha": .., "Time": ..}).
        retry_substeps : 0 (default, off: ASWING_Modified re-solves failed steps itself, TSPLIT).
            n > 1: a non-converged chunk is deleted and re-marched once in n*K steps of Dt/n
            (same control input); "Retried" records the outcome.
        record_initial : also record the initial point at t = 0.
        aero_flags : dict for Set_Aero_Flags (e.g. {"freeze_vl": True, "lag": False}), set in OPER.

        Open-loop controllers (Python): a UserController with schedule(times) -> {name: array}
        gets its commands written once to a time-history control file (one row per step, applied
        at the step end time). If it also has takeover(channels) -> bool, it is checked after every
        step; from the first True on, WingLoop calls step() every chunk (closed loop). The check
        is per step, so no step has to be rolled back.

        Returns True if every recorded step converged and the run reached N steps.
        """
        Dt, N, K = float(Dt), int(N), int(K)
        if N < 1 or K < 1:
            raise ValueError("N and K must be >= 1")
        if N > self.max_time_points:
            raise ValueError(f"N = {N} > {self.max_time_points}: ASWING stores at most NPNTX - 3 time points "
                             f"(DIMEN.INC NPNTX = 10001). Split the run, or raise WingLoop.max_time_points "
                             f"for a build with a larger NPNTX.")
        if self.WingLoop_LogFile is None or (self._recording_N is not None and self._recording_N < N + 1):
            self.Configure_Recording(N_steps=N + 1)
        n_calls = (N + K - 1) // K
        if self.PyControl is not None and hasattr(self.PyControl, "prepare_run"):
            self.PyControl.prepare_run(n_calls)
        self.metadata.update({"Dt": Dt, "N": N, "K": K, "retry_substeps": retry_substeps,
                              "record_initial": record_initial, "wingloop_version": __version__,
                              "wingloop_git": _git_commit(), "host": socket.gethostname(),
                              "python": sys.version.split()[0], "platform": platform.platform(),
                              "start_time": time.strftime("%Y-%m-%d %H:%M:%S")})

        user = getattr(self.PyControl, "python_controller_instance", None)
        takeover_state = {"open_loop": bool(user is not None and hasattr(user, "schedule")),
                          "takeover_time": None}
        schedule_file = self.input_filename + "_schedule"
        if takeover_state["open_loop"]:
            times = Dt * np.arange(1, N + 1)
            sched = user.schedule(times)
            python2text_schedule(schedule_file, times, {k: v for k, v in sched.items()
                                                        if _ASWING_COMMAND_RE.match(k)})
            self.metadata["open_loop_schedule"] = True

        n_before = len(self.WingLoop_LogFile["ModelVariables"]["IsConverged"]["values"])
        t_wall = time.time()
        self.sim_status = "running"
        step_ok, stopped = True, False
        self.count = 0
        try:
            self._enter_unsteady_oper(aero_flags)
            p = self._read_point()                                     # initial point, t = 0
            if record_initial:
                self._commit(p, 0.0)
            self._update_atmosphere(p)
            while self.count < N:
                k = min(K, N - self.count)
                t_now = self.count * Dt
                commands = None
                if self.fixed_stick:
                    control_file = None
                else:
                    commands, aswing_cmd = self._controller_commands(p, t_now, Dt * k, takeover_state)
                    if commands is None:
                        control_file = schedule_file
                        commands = {k_: float(np.asarray(v)[self.count + k - 1]) for k_, v in sched.items()}
                    else:
                        python2text(self.input_filename, aswing_cmd)
                        control_file = self.input_filename
                    if self.print_output:
                        print("[WingLoop] step =", self.count, "; control:", commands)
                out = self.ASW_handler.march(Dt, k, control_file=control_file, append=self.count > 0)
                restarts, tsplits = len(_RESTART_RE.findall(out)), len(_TSPLIT_RE.findall(out))
                p_new = self._read_point()
                retried = 0
                if not p_new["converged"] and retry_substeps and retry_substeps > 1:
                    n_sub = int(retry_substeps) * k
                    print(f"[WingLoop] t = {(self.count + k) * Dt:g} s did not converge: retrying in "
                          f"{n_sub} steps of {Dt / retry_substeps:g} s")
                    self.ASW_handler.send_command_and_receive("-")       # delete the failed point
                    out = self.ASW_handler.march(Dt / retry_substeps, n_sub, control_file=control_file,
                                                 append=self.count > 0)
                    restarts += len(_RESTART_RE.findall(out))
                    tsplits += len(_TSPLIT_RE.findall(out))
                    p_new = self._read_point()
                    retried = 1 if p_new["converged"] else 2
                    print(f"[WingLoop] retry {'converged' if retried == 1 else 'did not converge'}")
                self.count += k
                p = p_new
                self._commit(p, self.count * Dt, restarts, tsplits, retried, commands)
                if self.WingLoop_IsPlotting and self.LivePlot:
                    self.WingLoop_Liveplot.update(self.WingLoop_LogFile)
                if not p["converged"]:
                    step_ok = False
                    if stop_if_notconverged:
                        stopped = True
                        self.sim_status = f"stopped: not converged at step {self.count} (t = {self.count * Dt:g} s)"
                        break
                if stop_callback is not None and stop_callback(self):
                    stopped = True
                    self.sim_status = f"stopped by stop_callback at step {self.count}"
                    break
                if stop_condition is not None and stop_condition(self._channels(p, self.count * Dt)):
                    stopped = True
                    self.sim_status = f"stopped by stop_condition at step {self.count}"
                    break
                self._update_atmosphere(p)
            else:
                self.sim_status = "completed" if step_ok else "completed with non-converged steps"
        except ASWINGHangError as exc:
            self.sim_status = f"hung at step {self.count}: {str(exc).splitlines()[0]}"
            step_ok, stopped = False, True
        except ASWINGCrashError as exc:
            self.sim_status = f"crashed at step {self.count}: {str(exc).splitlines()[0]}"
            step_ok, stopped = False, True
        finally:
            self.metadata.update({"wall_time_s": time.time() - t_wall, "status": self.sim_status,
                                  "steps_done": self.count,
                                  "takeover_time": takeover_state["takeover_time"]})
        print(f"[WingLoop] {self.sim_status} ({self.count}/{N} steps, {time.time() - t_wall:.2f} s)")
        # the initial point (a loaded trim, not re-solved in this session) does not count
        conv = self.WingLoop_LogFile["ModelVariables"]["IsConverged"]["values"]
        allconverged = all(conv[n_before + (1 if record_initial else 0):])
        return bool(allconverged and step_ok and not stopped)

    def Performing_K_iterations_ASWING(self, Dt_aswing, K_aswing, custom_timer=None, retry_substeps=0):
        """V2 entry point, kept for compatibility: use Time_Transient_Simulation."""
        raise NotImplementedError("WingLoop 3.0 marches inside Time_Transient_Simulation "
                                  "(Performing_K_iterations_ASWING was a V2 internal).")

    def Define_Sim_Settings(self):
        """Placeholder (V2)."""
        pass

    # ================================================================== output
    def Outputting_The_State_File(self, statefile_filename=None):
        """Write the time history as an ASWING .state file (main menu HSAV), e.g. for videos."""
        name = (statefile_filename or self.WingLoop_LogFile["ModelName"] or "wingloop")
        if not name.endswith(".state"):
            name += ".state"
        _check_filename(name, "State file")
        if os.path.exists(name):
            os.remove(name)
        h = self.ASW_handler
        h.send_command_and_receive("")                          # leave OPER (main menu)
        if h.sync == "prompt":
            out, _ = h.send_command_and_receive("HSAV " + name, expect=r"Proceed to save state data\?  Y$")
            if out.rstrip().endswith("Y") and "Proceed to save" in out[-200:]:
                h.send_command_and_receive("Y")
        else:
            h.send_command_and_receive("HSAV")
            h.send_writefile_command_and_receive(name)
        if not os.path.exists(name):
            raise RuntimeError(f"ASWING did not write {name}")
        print("[WingLoop] Saved -> " + name)
        return name

    def Outputting_The_Results(self, custom_filename=None, fmt="npz", compress=True):
        """Save the record (default .npz; fmt="json" for the V2 .json.gz) and, if plotting
        is on, the figure as .pdf. Returns the data file path."""
        if custom_filename is None:
            custom_filename = self.WingLoop_LogFile["ModelName"] or "wingloop"
        self.WingLoop_LogFile["Metadata"] = dict(self.metadata)
        path = export_data_dict(self.WingLoop_LogFile, custom_filename, compress=compress, fmt=fmt)
        if self.WingLoop_IsPlotting:
            self.WingLoop_Liveplot.update(self.WingLoop_LogFile)
            self.WingLoop_Liveplot.export(custom_filename + ".pdf")
        return path

    def Closing_WingLoop(self, removefiles=True):
        """Quit ASWING, release the controller, remove exchange files, restore the caller's folder."""
        try:
            if removefiles:
                for fname in (self.output_filename, self.input_filename, self.input_filename + "_schedule"):
                    if os.path.exists(fname):
                        os.remove(fname)
            if self.WingLoop_IsPlotting and self.WingLoop_Liveplot is not None:
                self.WingLoop_Liveplot.close()
            if self.ASW_handler is not None:
                if self.ASW_handler.is_alive():
                    try:
                        self.ASW_handler.send_command_and_receive("", timeout=30)
                    except (ASWINGHangError, ASWINGCrashError):
                        pass
                self.ASW_handler.quit_and_close_aswing()
            if self.PyControl is not None:
                self.PyControl.terminate()
        finally:
            if getattr(self, "initial_path", None) and os.path.isdir(self.initial_path):
                os.chdir(self.initial_path)
        print("[WingLoop] Closed WingLoop correctly!")
