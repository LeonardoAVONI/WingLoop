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

The same PID (Kp 100, Ki 100, Kd 20 on pitch, acting on F2) is implemented by every
backend; run_all_backends.py runs them all and compares the results.
"""

import os
import sys
import time
import json
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


def run(backend="py", aswing=None, aswing_alias=None, N=300, Dt=0.01, K=1,
        sim_directory=None, out_name=None, plot=False, video=False, build="stable"):
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

    WL = WingLoop()
    WL.Launch_WingLoop_Control(cntrl_directory=None if cdir is None else os.path.join(HERE, cdir),
                               cntrl_filename=cfile, timestep=Dt * K,
                               precomputed_filename=reference if cdir is not None else None, **opts)
    if plot:
        WL.InitializePlot(liveplot=True, plot_variables=["earth X", "earth Z", "Elev.", "Alpha", "F2", "Velocity"],
                          plot_sim_time=N * Dt, plot_refreshtime=1, plot_size=(16, 10), N_steps=N)
    WL.Launch_ASWING(aswing_fullpath=aswing, aswing_alias=None if aswing else aswing_alias,
                     sim_directory=sim_directory, asw_filename=GEOMETRY, print_output=False)
    WL.Deactivate_Graphics()
    for f in CASE_FILES + (["00_Pitch_PID.con"] if backend == "native" else []):
        WL.Load_Files(f)

    t_start = time.time()
    converged = WL.Time_Transient_Simulation(Dt, N, K, record_initial=True,
                                             constraint_keys=["f", "c", "l"] if backend == "native" else None)
    t_end = time.time()
    WL.metadata["backend"] = backend
    WL.metadata["build"] = build
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
    timings = {"backend": backend, "build": build, "overhead_s": t_start - t_first + startup_in_run, "simulation_s": sim_s,
               "startup_in_run_s": startup_in_run, "ms_per_step": sim_s / N * 1000,
               "converged": converged, "status": WL.sim_status}
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
    ap.add_argument("--K", type=int, default=1)
    ap.add_argument("--sim-directory", default=None)
    ap.add_argument("--out", default=None, help="result name (without extension)")
    ap.add_argument("--timings-json", default=None, help="write the timings to this JSON file")
    ap.add_argument("--plot", action="store_true", help="live plot")
    ap.add_argument("--video", action="store_true", help="also write a .state file and a video")
    a = ap.parse_args()
    path, timings = run(a.backend, a.aswing, a.alias, a.N, a.Dt, a.K, a.sim_directory, a.out, a.plot, a.video,
                        build=a.build)
    timings["result"] = path
    if a.timings_json:
        with open(a.timings_json, "w") as f:
            json.dump(timings, f, indent=2)
