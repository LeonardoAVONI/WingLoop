# =============================================================================
# WingLoop — convergence_study.py (spatial and temporal convergence of the test aircraft)
# =============================================================================
# Copyright (c) 2026 Leonardo Avoni (avonileonardo@gmail.com)
# Part of WingLoop (CC BY-NC-SA 4.0).
# =============================================================================
"""
Spatial and temporal convergence of the WingLoop test aircraft (Murua HALE), for the
thesis WingLoop chapter (tab:wingloop_spatial_convergence, tab:wingloop_time_convergence).

    python convergence_study.py spatial  [--build stable] [--out DIR]
    python convergence_study.py temporal [--build stable] [--out DIR] [--T 3]

spatial:  level-flight trim at 30 m/s (make_trim.py recipe, then the free-flight re-solve)
          with the structural node counts of every beam scaled by FACTORS (wing and
          tailplane kept even, so the root double node splits them symmetrically).
temporal: fixed-stick WingLoop run from the trim (default mesh) through a 1-cosine gust
          (type 5: entry at X = 0, H = 24 m, i.e. L_g = 48 m, 3 m/s) for DTS; values at
          t = 0.7 s (the thesis table) and the peaks over the run.
Monitored: elevator F2, pitch theta, engine thrust E1, wing root bending moment (WRBM:
moment component 1 of the wing node at y = 0, beam axes) and tip deflection (mean z of the
two wing tips minus their jig z, 1.04189 m), plus the earth Z for the temporal study.
Errors are relative to the finest case. Results: DIR/<study>.json and <study>.md.
"""

import os
import re
import sys
import json
import shutil
import argparse
import subprocess

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import wingloop_testrun as wt                                      # noqa: E402

ALIASES = {"stable": "aswing_stable", "modified": "aswing_modified"}
BEAMS = ["Wing", "Horizontal Stabilizer", "Vertical Stabilizer", "Fuselage"]
DEFAULT_NODES = [48, 16, 12, 25]                  # structural nodes per beam (NODE menu, S)
FACTORS = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0]
DTS = [0.1, 0.05, 0.02, 0.01, 1 / 120, 0.005, 0.002]
Z_TIP_JIG = 1.04189                               # jig z of the wing tips (sensor 2 of the .asw)
TRIM = ["%", "f", "9 3", "17 1", "20 5", "23 32", "24 6", "d", "x", "W p_trim",
        "r", "%", "f", "c", ".", "d", "x", "W p_free"]


def nodes_for(factor):
    n = [max(3, int(round(factor * d))) for d in DEFAULT_NODES]
    for b in (0, 1):
        n[b] = max(4, 2 * int(round(factor * DEFAULT_NODES[b] / 2)))
    return n


def read_point(path):
    """Scalars and the distributed states (IITOT, 18) of a point file."""
    text = open(path).read()
    head, _, rest = text.partition("STARTEDPRINTINGSTATES")
    num = lambda k: float(re.search(re.escape(k) + r"\s*:\s*([-0-9.E+]+)", head).group(1))
    iitot = int(re.search(r"IITOT:\s*(\d+)", head).group(1))
    vals = np.array(rest.split("IITOT", 1)[1].split()[:18 * iitot], float)   # first block: 18*IITOT values
    return {"converged": "Status: Converged" in head, "theta": num("Elev."), "F2": num("Flap 2"),
            "E1": num("Peng 1"), "Q": vals[:18 * iitot].reshape(iitot, 18)}


def loads(Q, n_wing):
    """WRBM [N m] and tip deflection [m] from the distributed states (wing = first n_wing nodes)."""
    W = Q[:n_wing]
    root = np.argmin(np.abs(W[:, 1]))
    tips = [np.argmin(W[:, 1]), np.argmax(W[:, 1])]
    return float(W[root, 6]), float(np.mean(W[tips, 2]) - Z_TIP_JIG)


def spatial(exe, out):
    rows = []
    for fct in FACTORS:
        nodes = nodes_for(fct)
        case = os.path.join(out, f"spatial_x{fct:g}")
        shutil.rmtree(case, ignore_errors=True)
        os.makedirs(case)
        shutil.copy(os.path.join(HERE, "aswing_geometry_stable", wt.GEOMETRY), case)
        cmds = ["PLPA", "G", "", "NODE"] + sum([[f"B {b}", f"I {n}"] for b, n in enumerate(nodes, 1)], []) \
            + ["S", "", "", "oper", "!V 30"] + TRIM + ["", "quit"]
        with open(os.path.join(case, "in"), "w") as f:
            f.write("\n".join(cmds) + "\n")
        env = dict(os.environ, GFORTRAN_UNBUFFERED_ALL="1")
        with open(os.path.join(case, "in")) as fin, open(os.path.join(case, "log"), "w") as flog:
            subprocess.run([exe, wt.GEOMETRY.replace(".asw", "")], cwd=case, stdin=fin, stdout=flog,
                           stderr=subprocess.STDOUT, env=env, timeout=600)
        log = open(os.path.join(case, "log")).read()
        blk = log.rsplit("structural  circulation", 1)[1]
        circ = sum(int(m) for m in re.findall(r"\d+\s+(?:Wing|Horizontal Stabilizer|Vertical Stabilizer)\s+\d+\s+(\d+)", blk))
        p = read_point(os.path.join(case, "p_free"))
        wrbm, tip = loads(p["Q"], nodes[0])
        rows.append({"factor": fct, "nodes": nodes, "structural": sum(nodes), "circulation": circ,
                     "converged": p["converged"], "F2": p["F2"], "theta": p["theta"], "E1": p["E1"],
                     "WRBM": wrbm, "tip": tip})
        print(f"[spatial] x{fct:g} {sum(nodes)}/{circ} nodes: F2 {p['F2']:.3f} theta {p['theta']:.4f} "
              f"E1 {p['E1']:.3f} WRBM {wrbm:.2f} tip {tip:.4f} {'' if p['converged'] else 'NOT CONVERGED'}",
              flush=True)
    keys = ["F2", "theta", "E1", "WRBM", "tip"]
    ref = rows[-1]
    for r in rows:
        r["max_err_pct"] = max(abs(r[k] / ref[k] - 1) * 100 for k in keys)
    lines = ["| structural | circulation | F2 [deg] | pitch [deg] | E1 [N] | WRBM [N m] | tip defl. [m] | max err |",
             "|---|---|---|---|---|---|---|---|"]
    lines += [f"| {r['structural']} | {r['circulation']} | {r['F2']:.3f} | {r['theta']:.3f} | {r['E1']:.3f} | "
              f"{r['WRBM']:.3f} | {r['tip']:.3f} | {r['max_err_pct']:.2f} % |" for r in rows]
    return rows, lines


def temporal(exe, out, build, T):
    rows = []
    for dt in DTS:
        N = int(round(T / dt))
        case = os.path.join(out, f"temporal_dt{dt:.5f}")
        shutil.rmtree(case, ignore_errors=True)
        shutil.copytree(os.path.join(HERE, f"aswing_geometry_{build}"), case,
                        ignore=shutil.ignore_patterns("result*", "*.npz", "controller_config.json"))
        with open(os.path.join(case, "01_1minus_cosine.gust"), "w") as f:
            f.write(" 5\t! 1-cosine vertical gust\n 0\t! gust entry X\n 0\t! y slope\n 24\t! H\n 3\t! Wmax\n 0\n")
        tj = os.path.join(case, "timings.json")
        subprocess.run([sys.executable, os.path.join(HERE, "wingloop_testrun.py"), "fixed", "--build", build,
                        "--aswing", exe, "--N", str(N), "--Dt", repr(dt), "--sim-directory", case, "--out", "result",
                        "--timings-json", tj], cwd=HERE, stdout=open(os.path.join(case, "log"), "w"),
                       stderr=subprocess.STDOUT)
        from WingLoop_Library import import_data_dict
        d = import_data_dict(os.path.join(case, "result.npz"))
        V = d["ModelVariables"]
        t = np.asarray(V["Time"]["values"], float)
        S = np.asarray(d["ModelStates"], float)
        iitot = d["ModelParameters"]["IITOT"]
        L = np.array([loads(s[:18 * iitot].reshape(iitot, 18), DEFAULT_NODES[0]) for s in S])
        ser = {"Z": np.asarray(V["earth Z"]["values"], float), "theta": np.asarray(V["Elev."]["values"], float),
               "WRBM": L[:, 0], "tip": L[:, 1]}
        i07 = int(np.argmin(np.abs(t - 0.7)))
        conv = np.asarray(V["IsConverged"]["values"][1:], bool)
        row = {"dt": dt, "N": N, "t07": float(t[i07]), "all_converged": bool(conv.all()),
               "first_unconverged_t": float(t[1:][~conv][0]) if (~conv).any() else None}
        for k, v in ser.items():
            row[f"{k}_07"] = float(v[i07])
            row[f"{k}_max"], row[f"{k}_tmax"] = float(v.max()), float(t[v.argmax()])
            row[f"{k}_min"], row[f"{k}_tmin"] = float(v.min()), float(t[v.argmin()])
        np.savez(os.path.join(case, "series.npz"), t=t, **ser)
        rows.append(row)
        print(f"[temporal] dt {dt:.5f}: at 0.7 s Z {row['Z_07']:.4f} theta {row['theta_07']:.4f} "
              f"WRBM {row['WRBM_07']:.2f} tip {row['tip_07']:.4f}; WRBM max {row['WRBM_max']:.2f} at "
              f"{row['WRBM_tmax']:.3f} s {'' if row['all_converged'] else 'NOT CONVERGED'}", flush=True)
    return finish_temporal(rows)


def finish_temporal(rows):
    """Errors against the finest run whose every step converged (a failed step, e.g. at
    dt = 0.002 s on the stable build, corrupts the rest of that run)."""
    ref = [r for r in rows if r["all_converged"]][-1]
    keys07 = ["Z_07", "theta_07", "WRBM_07", "tip_07"]
    keyspk = ["theta_max", "theta_min", "WRBM_max", "WRBM_min", "tip_max", "tip_min"]
    for r in rows:
        r["max_err_07_pct"] = max(abs(r[k] / ref[k] - 1) * 100 for k in keys07)
        r["max_err_peaks_pct"] = (max(abs(r[k] / ref[k] - 1) * 100 for k in keyspk) if r["all_converged"]
                                  else float("nan"))
    lines = ["| dt [s] | steps to 0.7 s | Z [m] | pitch [deg] | WRBM [N m] | tip defl. [m] | max err (0.7 s) | "
             "max err (peaks) |", "|---|---|---|---|---|---|---|---|"]
    lines += [f"| {r['dt']:.5f} | {int(round(0.7 / r['dt']))} | {r['Z_07']:.4f} | {r['theta_07']:.4f} | "
              f"{r['WRBM_07']:.3f} | {r['tip_07']:.4f} | {r['max_err_07_pct']:.2f} % | "
              + (f"{r['max_err_peaks_pct']:.2f} % |" if r["all_converged"]
                 else f"step failed at t = {r['first_unconverged_t']:g} s |") for r in rows]
    lines += ["", f"Reference: dt = {ref['dt']:g} s (finest run with every step converged)."]
    return rows, lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("study", choices=["spatial", "temporal"])
    ap.add_argument("--build", default="stable", choices=sorted(ALIASES))
    ap.add_argument("--aswing", default=None)
    ap.add_argument("--out", default=os.path.join(HERE, "results_convergence"))
    ap.add_argument("--T", type=float, default=3.0, help="temporal: simulated time [s]")
    a = ap.parse_args()
    from WingLoop_Library.Aswing_Director import resolve_aswing_executable
    exe = resolve_aswing_executable(a.aswing, None if a.aswing else ALIASES[a.build])
    os.makedirs(a.out, exist_ok=True)
    rows, lines = spatial(exe, a.out) if a.study == "spatial" else temporal(exe, a.out, a.build, a.T)
    head = [f"# {a.study} convergence, test aircraft ({a.build} build, `{exe}`)", "",
            "Errors relative to the finest case." + (" Default mesh: x1." if a.study == "spatial" else ""), ""]
    with open(os.path.join(a.out, f"{a.study}.md"), "w") as f:
        f.write("\n".join(head + lines) + "\n")
    with open(os.path.join(a.out, f"{a.study}.json"), "w") as f:
        json.dump(rows, f, indent=2)
    print("\n".join(lines))


if __name__ == "__main__":
    main()
