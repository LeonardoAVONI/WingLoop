# =============================================================================
# WingLoop — wingloop_testrun.py
# =============================================================================
# Copyright (c) 2024-2026 Leonardo Avoni (avonileonardo@gmail.com)
#
# This file is part of WingLoop.
# WingLoop is licensed under CC BY-NC-SA 4.0 (Non-Commercial use only).
# Full license: https://creativecommons.org/licenses/by-nc-sa/4.0/
#
# If you use WingLoop in academic work, please cite:
#   Avoni et al., "Enhancing ASWING Flight Dynamics Simulations with
#   Closed-Loop Control for Flexible Aircraft," AIAA 2025-3425.
# =============================================================================
"""
WingLoop 3.0 test case: one closed-loop run of the test aircraft through a 1-cosine gust.

    python wingloop_testrun.py [backend] [--build stable|modified] [--aswing PATH] [--N 300] [--plot] [--video]

build: the aircraft version, one per ASWING build (their .pnt/.set/.state files are not
       interchangeable): aswing_geometry_stable/ (ASWING_Stable, alias aswing_stable) or
       aswing_geometry_modified/ (ASWING_Modified, alias aswing_modified). Both are the same
       geometry trimmed at 30 m/s by make_trim.py; trim_reference.json holds the trim the
       controllers regulate to.

backend: py (default) | mat | sim (Simulink V1) | fmu (Simulink FMU, as committed) |
         fmu_rebuild (re-export the FMU first) | simv2 (Simulink V2) | fixed (no controller) |
         native (ASWING's own controller, 00_Pitch_PID.con: WingLoop runs fixed stick and
         sets the OPER constraint table to "f c l" (free flight, controls pinned, control law on))

The same PID on pitch, acting on F2, is implemented by every backend; run_all_backends.py
runs them all and compares the results. --scenario picks its gains (SCENARIOS below); the
thesis WingLoop chapter uses v1_k1 for every run and v1_k10 only for the Python K = 10 run:
    v1_k1   Kp 7, Ki 30 1/s, Kd 8 s in deg/deg, K = 1 (default)
    v1_k10  Kp 2, Ki 0.5 1/s, Kd 0.5 s in deg/deg, K = 10 (controller every 10 steps)
The gains are written, in deg/rad, into controller_config.json in the run folder (trim
references + gains), which every backend's UserController reads; the native backend gets a
.con file with the same gains (make_con).
"""

import os
import sys
import time
import json
import re
import math
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
from WingLoop_Library import WingLoop

BACKENDS = {
    #  name          (control_dir,           control_file,                    options)
    "py":          ("python_controller",   "python_test_controller.py",     {}),
    "mat":         ("matlab_controller",   "UserController.m",              {}),
    "sim":         ("simulink_controller", "simulink_test_controller.slx",  {}),
    "fmu":         ("simulink_controller", "simulink_test_controller.fmu",  {"rebuild_fmu_file": False}),
    "fmu_rebuild": ("simulink_controller", "simulink_test_controller.fmu",  {"rebuild_fmu_file": True}),
    "simv2":       ("simulink_controller", "simulink_test_controller.slx",  {"simulink_mode": "v2"}),
    "fixed":       (None, None, {}),
    "native":      (None, None, {}),
}
GEOMETRY = "wingloop_test_aircraft.asw"
CASE_FILES = ["01_1minus_cosine.gust", "wingloop_test_aircraft.pnt", "wingloop_test_aircraft.set",
              "wingloop_test_aircraft.state"]
BUILDS = {"stable": "aswing_stable", "modified": "aswing_modified"}     # build -> default shell alias

# PID gains per scenario: (Kp, Ki, Kd, unit of the error) and the default K. "deg" gains act on
# the pitch error in degrees; they are converted to deg/rad, the unit every backend uses.
SCENARIOS = {
    "v1_k1":  {"gains": (7.0, 30.0, 8.0),     "error_unit": "deg", "K": 1},
    "v1_k10": {"gains": (2.0, 0.5, 0.5),      "error_unit": "deg", "K": 10},
}
CON_TEMPLATE = os.path.join(HERE, "aswing_geometry_stable", "00_Pitch_PID.con")


def scenario_gains(scenario):
    """(Kp, Ki, Kd) of a scenario in deg/rad (pitch error in rad -> F2 increment in deg)."""
    sc = SCENARIOS[scenario]
    f = 180.0 / math.pi if sc["error_unit"] == "deg" else 1.0
    return tuple(g * f for g in sc["gains"])


def make_config(sim_directory, scenario):
    """controller_config.json = trim_reference.json + the scenario's gains; returns its path."""
    with open(os.path.join(sim_directory, "trim_reference.json")) as f:
        cfg = json.load(f)
    cfg["Kp"], cfg["Ki"], cfg["Kd"] = scenario_gains(scenario)
    cfg["scenario"] = scenario
    path = os.path.join(sim_directory, "controller_config.json")
    with open(path, "w") as f:
        json.dump(cfg, f, indent=2)
    return path


def make_con(sim_directory, scenario):
    """ASWING control-law file with the scenario's gains (P on pitch, I on its integral, D on
    the pitch rate), from the template 00_Pitch_PID.con; returns its file name."""
    lines = open(CON_TEMPLATE).read().splitlines()
    gains = list(native_gains(scenario))
    out = []
    for ln in lines:
        if gains and re.match(r"^\s*[-+0-9.eE]+\s*!\s*gain value", ln):
            out.append(f"   {gains.pop(0):.10g}                   ! gain value ({scenario})")
        else:
            out.append(ln)
    if gains:
        raise RuntimeError(f"{CON_TEMPLATE}: expected three 'gain value' lines")
    out = [f"# Generated by wingloop_testrun.make_con for scenario {scenario}: gains in F2 per pitch",
           "# error in the same angle unit (deg/deg = rad/rad), i.e. the WingLoop deg/rad gains x pi/180."] + out
    name = f"00_Pitch_PID_{scenario}.con"
    with open(os.path.join(sim_directory, name), "w") as f:
        f.write("\n".join(out) + "\n")
    return name


def native_gains(scenario):
    """Gains written into the .con file. ASWING's control law maps the pitch error to F2 in the
    same angle unit (rad -> rad, i.e. deg -> deg), so its gains are the deg/rad gains times
    pi/180 (checked 2026-11-04 on v1_k1: native with 7, 30, 8 matches the Python PID within
    0.03 deg of F2; with 401, 1719, 458 it held pitch flat, 57.3 times too stiff)."""
    return tuple(g * math.pi / 180.0 for g in scenario_gains(scenario))


def _timed(obj, name, acc, key):
    """Replace obj.name by a wrapper adding its wall time to acc[key] (time split of a run)."""
    f = getattr(obj, name)

    def wrapper(*args, **kwargs):
        t0 = time.perf_counter()
        try:
            return f(*args, **kwargs)
        finally:
            acc[key] += time.perf_counter() - t0
    setattr(obj, name, wrapper)


def run(backend="py", aswing=None, aswing_alias=None, N=300, Dt=0.01, K=None,
        sim_directory=None, out_name=None, plot=False, video=False, build="stable", scenario="v1_k1"):
    """Run one backend; returns (result path, timings dict)."""
    t_first = time.time()
    aswing_alias = aswing_alias or BUILDS[build]
    sim_directory = os.path.abspath(sim_directory or os.path.join(HERE, f"aswing_geometry_{build}"))
    reference = os.path.join(sim_directory, "trim_reference.json")
    if not os.path.isfile(reference):
        raise FileNotFoundError(f"{reference} is missing: run make_trim.py {build} (the controllers "
                                f"must regulate to this aircraft version's trim)")
    out_name = out_name or f"testrun_{backend}"
    cdir, cfile, opts = BACKENDS[backend]
    K = SCENARIOS[scenario]["K"] if K is None else K
    config = make_config(sim_directory, scenario)
    con_file = make_con(sim_directory, scenario) if backend == "native" else None

    WL = WingLoop()
    WL.Launch_WingLoop_Control(cntrl_directory=None if cdir is None else os.path.join(HERE, cdir),
                               cntrl_filename=cfile, timestep=Dt * K,
                               precomputed_filename=config if cdir is not None else None, **opts)
    if plot:
        WL.InitializePlot(liveplot=True, plot_variables=["earth X", "earth Z", "Elev.", "Alpha", "F2", "Velocity"],
                          plot_sim_time=N * Dt, plot_refreshtime=1, plot_size=(16, 10), N_steps=N)
    WL.Launch_ASWING(aswing_fullpath=aswing, aswing_alias=None if aswing else aswing_alias,
                     sim_directory=sim_directory, asw_filename=GEOMETRY, print_output=False)
    WL.Deactivate_Graphics()
    # time split of the march: ASWING solving (march), writing + parsing the point file,
    # controller call, live plot; the rest of the loop is WingLoop bookkeeping
    split = {"aswing_march_s": 0.0, "read_point_s": 0.0, "controller_s": 0.0, "plot_s": 0.0}
    _timed(WL.ASW_handler, "march", split, "aswing_march_s")
    _timed(WL, "_read_point", split, "read_point_s")
    _timed(WL, "_controller_commands", split, "controller_s")
    if plot and getattr(WL, "WingLoop_Liveplot", None) is not None:
        _timed(WL.WingLoop_Liveplot, "update", split, "plot_s")
    for f in CASE_FILES + ([con_file] if backend == "native" else []):
        WL.Load_Files(f)

    t_start = time.time()
    converged = WL.Time_Transient_Simulation(Dt, N, K, record_initial=True,
                                             constraint_keys=["f", "c", "l"] if backend == "native" else None)
    t_end = time.time()
    split = dict(split)          # freeze: the result export below calls the plot again
    WL.metadata["backend"] = backend
    WL.metadata["build"] = build
    WL.metadata["scenario"] = scenario
    WL.metadata["pid_gains_deg_per_rad"] = list(scenario_gains(scenario))
    # one-off costs paid at the first controller step (they need the state length):
    # Simulink V2 model build + compile, FMU re-export. Reported as start-up, not per step.
    ctrl = WL.PyControl
    startup_in_run = 0.0
    if ctrl is not None:
        if getattr(ctrl, "v2_controller", None) is not None and ctrl.v2_controller.startup_time:
            startup_in_run = ctrl.v2_controller.startup_time
        startup_in_run += getattr(ctrl, "fmu_rebuild_time", 0.0) or 0.0
    path = WL.Outputting_The_Results(out_name)
    path = os.path.join(sim_directory, path) if not os.path.isabs(path) else path
    if video:
        WL.Outputting_The_State_File(out_name)
    WL.Closing_WingLoop(removefiles=True)
    sim_s = t_end - t_start - startup_in_run
    split["controller_s"] -= startup_in_run       # the one-off start-up ran inside the first controller call
    timings = {"backend": backend, "build": build, "scenario": scenario, "K": K, "overhead_s": t_start - t_first + startup_in_run, "simulation_s": sim_s,
               "startup_in_run_s": startup_in_run, "ms_per_step": sim_s / N * 1000,
               "converged": converged, "status": WL.sim_status, "plot": bool(plot), **split,
               "other_s": sim_s - sum(split.values()),
               "newton_restarts": int(sum(WL.WingLoop_LogFile["ModelVariables"]["NewtonRestarts"]["values"])),
               "tsplits": int(sum(WL.WingLoop_LogFile["ModelVariables"]["TSplits"]["values"]))}
    print(f"[wingloop_testrun] {backend}: overhead {timings['overhead_s']:.2f} s (of which {startup_in_run:.2f} s "
          f"at the first step), simulation {sim_s:.2f} s ({timings['ms_per_step']:.1f} ms/step), {WL.sim_status}")

    if video:
        from WingLoop_Library import VideoPlot_Generate_postscript_file, Generate_Analysis_Videos
        ps = VideoPlot_Generate_postscript_file(geometry_filename=GEOMETRY, reference_folder=sim_directory,
                                                input_files=CASE_FILES[1:3] + [out_name + ".state"],
                                                video_folder=".", aswing_path=aswing, aswing_alias=aswing_alias,
                                                movie_time_limit=N * Dt, Dt=Dt)
        Generate_Analysis_Videos(ps, Dt=Dt, transpose=1, quality="medium")
    return path, timings


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("backend", nargs="?", default="py", choices=sorted(BACKENDS))
    ap.add_argument("--aswing", default=None, help="ASWING executable or its folder (default: alias)")
    ap.add_argument("--build", default="stable", choices=sorted(BUILDS), help="aircraft version / ASWING build")
    ap.add_argument("--alias", default=None, help="shell alias used when --aswing is not given (default per build)")
    ap.add_argument("--N", type=int, default=300)
    ap.add_argument("--Dt", type=float, default=0.01)
    ap.add_argument("--K", type=int, default=None, help="controller every K steps (default: the scenario's)")
    ap.add_argument("--scenario", default="v1_k1", choices=sorted(SCENARIOS), help="PID gains (see above)")
    ap.add_argument("--sim-directory", default=None)
    ap.add_argument("--out", default=None, help="result name (without extension)")
    ap.add_argument("--timings-json", default=None, help="write the timings to this JSON file")
    ap.add_argument("--plot", action="store_true", help="live plot")
    ap.add_argument("--video", action="store_true", help="also write a .state file and a video")
    a = ap.parse_args()
    path, timings = run(a.backend, a.aswing, a.alias, a.N, a.Dt, a.K, a.sim_directory, a.out, a.plot, a.video,
                        build=a.build, scenario=a.scenario)
    timings["result"] = path
    if a.timings_json:
        with open(a.timings_json, "w") as f:
            json.dump(timings, f, indent=2)
