# =============================================================================
# WingLoop — make_trim.py (trim the test aircraft with a given ASWING build)
# =============================================================================
# Copyright (c) 2026 Leonardo Avoni (avonileonardo@gmail.com)
# Part of WingLoop (CC BY-NC-SA 4.0).
# =============================================================================
"""
Trim the WingLoop test aircraft and save the operating point, settings and state files
for one ASWING build.

    python make_trim.py stable   [--aswing PATH] [--out DIR] [--check-only]
    python make_trim.py modified [--aswing PATH] [--out DIR] [--check-only]

ASWING builds with different array sizes cannot read each other's .pnt/.set/.state files
("Dimensions have changed"): ASWING_Modified stores one more operating-point parameter
(KPTOT 120 against 119). The geometry (.asw), gust (.gust) and control-law (.con) files are
plain text and shared. This script makes the build-specific files of aswing_geometry_<build>/:

  1. level-flight trim at V = 30 m/s (operating point table, recipe of the
     20_Flexible_Models_and_Control project):
         %  f                free-flight defaults
         9 3                 Uz (alpha) <- z-acceleration = 0
         17 1                Theta     <- x-acceleration = 0
         20 5                F2        <- pitch acceleration = 0
         23 32               E1        <- climb angle = 0
         24 6                E2        <- yaw acceleration = 0  (symmetric thrust)
         d   x
  2. free-flight re-solve with the trimmed controls pinned:  r  %  f  c  .  d  x
  3. PSAV, SSAV, HSAV  ->  wingloop_test_aircraft.{pnt,set,state}
     (HSAV, not TSAV: TSAV saves the last 3 points and crashes ASWING with a single point)
  4. trim_reference.json: the trim values the test controllers regulate to
     (F2ref [deg], E2ref = engine setting, Pitchref [rad]), read by every backend's
     UserController through WingLoop's precomputed-file option.

The k-menu settings are the build's defaults (ASWING_Modified: frozen VL matrices, lag on).
--check-only trims and prints the result without writing files.
"""

import os
import re
import sys
import json
import math
import shutil
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
from WingLoop_Library.Aswing_Director import Aswing_Director, resolve_aswing_executable  # noqa: E402

NAME = "wingloop_test_aircraft"
ALIASES = {"stable": "aswing_stable", "modified": "aswing_modified"}
TABLE_PROMPT = r"\( ABCDFLPQSX\.\+- \): $"
TRIM_TOGGLES = ["9 3", "17 1", "20 5", "23 32", "24 6"]
SHARED = [NAME + ".asw", "01_1minus_cosine.gust", "00_Pitch_PID.con"]


def table(asw, keys):
    """Enter the OPER constraint table, send `keys`, leave it with d."""
    asw.send_command_and_receive("%", expect=TABLE_PROMPT)
    for k in keys:
        asw.send_command_and_receive(k, expect=TABLE_PROMPT)
    asw.send_command_and_receive("d")


def point(asw, fname="trim_point"):
    asw.write_point_file(fname)
    txt = open(fname).read()
    os.remove(fname)
    get = lambda k: float(re.search(re.escape(k) + r"\s*:\s*([-0-9.E+]+)", txt).group(1))
    return {"converged": "Status: Converged" in txt, "V": get("Velocity"), "alpha": get("Alpha"),
            "theta": get("Elev."), "F2": get("Flap 2"), "E1": get("Peng 1"), "E2": get("Peng 2")}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("build", choices=sorted(ALIASES))
    ap.add_argument("--aswing", default=None, help="ASWING executable or folder (default: alias of the build)")
    ap.add_argument("--out", default=None, help="output folder (default aswing_geometry_<build>)")
    ap.add_argument("--V", type=float, default=30.0)
    ap.add_argument("--check-only", action="store_true")
    a = ap.parse_args()
    exe = resolve_aswing_executable(a.aswing, None if a.aswing else ALIASES[a.build])
    out = os.path.abspath(a.out or os.path.join(HERE, f"aswing_geometry_{a.build}"))
    os.makedirs(out, exist_ok=True)
    for f in SHARED:
        src = os.path.join(HERE, "aswing_geometry_stable", f)
        if not os.path.exists(os.path.join(out, f)):
            shutil.copy(src, out)

    asw = Aswing_Director(aswing_path=exe, write_timeout=600)
    asw.start_aswing(directory=out, filename=NAME, print_output=False)
    for c in ("PLPA", "G", ""):
        asw.send_command_and_receive(c)
    asw.send_command_and_receive("oper")
    asw.send_command_and_receive(f"!V {a.V}")
    table(asw, ["f"] + TRIM_TOGGLES)
    asw.send_command_and_receive("x")
    trim = point(asw)
    print(f"[make_trim] {a.build} trim: {trim}")
    asw.send_command_and_receive("r")
    table(asw, ["f", "c", "."])
    asw.send_command_and_receive("x")
    free = point(asw)
    print(f"[make_trim] {a.build} free-flight re-solve: {free}")
    ok = trim["converged"] and free["converged"]
    if ok and not a.check_only:
        asw.send_command_and_receive("")                       # main menu
        for cmd, ext in (("PSAV", ".pnt"), ("SSAV", ".set"), ("HSAV", ".state")):
            path = os.path.join(out, NAME + ext)
            if os.path.exists(path):
                os.remove(path)
            asw.send_command_and_receive(f"{cmd} {NAME}{ext}", expect=r"Proceed to save state data\?  Y$")
            if "Proceed to save" in asw.last_output[-200:]:
                asw.send_command_and_receive("Y")
            if not os.path.exists(path):
                raise RuntimeError(f"{cmd} did not write {path}:\n{asw.last_output[-800:]}")
        ref = {"build": a.build, "aswing": exe, "V": free["V"], "alpha_deg": free["alpha"],
               "F2ref": free["F2"], "E2ref": free["E1"], "E1": free["E1"], "E2": free["E2"],
               "Pitchref": math.radians(free["theta"]), "theta_deg": free["theta"]}
        with open(os.path.join(out, "trim_reference.json"), "w") as f:
            json.dump(ref, f, indent=2)
        print(f"[make_trim] wrote {NAME}.pnt/.set/.state and trim_reference.json in {out}")
    asw.quit_and_close_aswing()
    os.chdir(HERE)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
