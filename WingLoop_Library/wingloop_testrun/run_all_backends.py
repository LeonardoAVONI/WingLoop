# =============================================================================
# WingLoop — run_all_backends.py (cross-check of the controller backends)
# =============================================================================
# Copyright (c) 2026 Leonardo Avoni (avonileonardo@gmail.com)
# Part of WingLoop (CC BY-NC-SA 4.0).
# =============================================================================
"""
Run the test case with every controller backend and check they give the same result.

    python run_all_backends.py [--build stable|modified|all] [--backends fixed,py,mat,sim,fmu_rebuild,fmu,simv2,native]
                               [--aswing PATH] [--N 300] [--out results_v3]

--build all (default) runs every backend on both aircraft versions (ASWING_Stable and
ASWING_Modified), each in results_v3/<build>/, with one report per build. The FMU is
re-exported for each build (fmu_rebuild, which bakes that build's trim references and
state length into it) before the "fmu" run uses it.

Every backend implements the same PID, so with the same inputs every backend must
command the same F2 and produce the same ASWING states. Each run is done in its own
process and its own copy of aswing_geometry/ (no shared exchange files). The report
(<out>/report.md, <out>/report.json) lists, against the Python run:

    max |dF2|        largest difference of the commanded elevator [deg]
    max |dstate|/rng largest difference over all ASWING states, divided by each state's range
    lag check        the same differences with the backend shifted by one step; a
                     backend that is one step late matches much better shifted (FAIL)

and the timings (overhead, ms per step; the "fixed" run is the fixed-stick baseline:
WingLoop's cost with no controller). "native" is ASWING's own PID (00_Pitch_PID.con)
and is not expected to match exactly: ASWING applies its control law inside the step
(implicit), WingLoop one step later (README, "WingLoop TestRun").

Pass criteria: max |dF2| < 1e-6 deg and max |dstate|/range < 1e-5 (round-off), and no
one-step lag. Exit code 1 if a backend fails.
"""

import os
import sys
import json
import shutil
import argparse
import subprocess

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOL_F2 = 1e-6
TOL_STATE = 1e-5


def load(path):
    from WingLoop_Library import import_data_dict
    d = import_data_dict(path)
    V = d["ModelVariables"]
    cmd_f2 = np.asarray(V.get("cmd_F2", {"values": []})["values"], dtype=float)
    return {"t": np.asarray(V["Time"]["values"], float), "F2": np.asarray(V["F2"]["values"], float),
            "cmdF2": cmd_f2, "S": np.asarray(d["ModelStates"], float),
            "Elev": np.asarray(V["Elev."]["values"], float), "conv": V["IsConverged"]["values"]}


def compare(ref, run, shift=0):
    """Max |dF2| (ASWING-applied F2) and max |dS|/range, run shifted by `shift` steps."""
    n = min(len(ref["F2"]), len(run["F2"])) - abs(shift)
    a = slice(0, n) if shift >= 0 else slice(-shift, -shift + n)
    b = slice(shift, shift + n) if shift >= 0 else slice(0, n)
    dF2 = float(np.max(np.abs(ref["F2"][a] - run["F2"][b]))) if n > 0 else np.nan
    m = min(ref["S"].shape[1], run["S"].shape[1])
    rng = np.ptp(ref["S"][:, :m], axis=0)
    rng[rng < 1e-12] = 1.0
    dS = float(np.max(np.abs(ref["S"][a, :m] - run["S"][b, :m]) / rng)) if n > 0 else np.nan
    return dF2, dS


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backends", default="fixed,py,mat,sim,fmu_rebuild,fmu,simv2,native")
    ap.add_argument("--build", default="all", choices=["stable", "modified", "all"])
    ap.add_argument("--aswing", default=None, help="ASWING executable or folder (only with one --build)")
    ap.add_argument("--N", type=int, default=300)
    ap.add_argument("--Dt", type=float, default=0.01)
    ap.add_argument("--out", default=os.path.join(HERE, "results_v3"))
    ap.add_argument("--reuse", action="store_true", help="only compare existing results")
    a = ap.parse_args()
    backends = [b.strip() for b in a.backends.split(",") if b.strip()]
    # stable last: the .fmu left in simulink_controller/ is then the stable build's (the default)
    builds = ["modified", "stable"] if a.build == "all" else [a.build]
    if a.aswing and len(builds) > 1:
        ap.error("--aswing needs a single --build")
    failed = False
    root = a.out
    for build in builds:
        a.out = os.path.join(root, build)
        failed |= run_build(a, build, backends)
    return 1 if failed else 0


ALIASES = {"stable": "aswing_stable", "modified": "aswing_modified"}


def run_build(a, build, backends):
    os.makedirs(a.out, exist_ok=True)
    from WingLoop_Library.Aswing_Director import resolve_aswing_executable
    exe = a.aswing or resolve_aswing_executable(aswing_alias=ALIASES[build])      # once, here
    geometry = os.path.join(HERE, f"aswing_geometry_{build}")

    timings = {}
    for b in backends:
        case = os.path.join(a.out, b)
        tj = os.path.join(a.out, f"{b}_timings.json")
        if not a.reuse:
            if os.path.isdir(case):
                shutil.rmtree(case)
            if os.path.exists(tj):
                os.remove(tj)            # never report a previous run's result
            shutil.copytree(geometry, case,
                            ignore=shutil.ignore_patterns("input", "output", "result*", "*.npz", "*.pdf",
                                                          "*.ps", "*.gif", "*.mp4", "*.webm", "video*"))
            print(f"\n===== {build}: {b} =====", flush=True)
            cmd = [sys.executable, os.path.join(HERE, "wingloop_testrun.py"), b, "--build", build, "--aswing", exe,
                   "--N", str(a.N), "--Dt", str(a.Dt), "--sim-directory", case, "--out", "result",
                   "--timings-json", tj]
            with open(os.path.join(a.out, f"{b}.log"), "w") as log:
                r = subprocess.run(cmd, cwd=HERE, stdout=log, stderr=subprocess.STDOUT)
            if r.returncode != 0:
                print(f"  {b}: FAILED to run (see {b}.log)")
        if os.path.exists(tj):
            with open(tj) as f:
                timings[b] = json.load(f)

    results = {b: load(timings[b]["result"]) for b in timings}
    rows, failed = [], False
    ref = results.get("py")
    for b in backends:
        if b not in results:
            rows.append({"backend": b, "verdict": "NOT RUN"})
            failed = True
            continue
        t = timings[b]
        row = {"backend": b, "status": t["status"], "overhead_s": round(t["overhead_s"], 2),
               "ms_per_step": round(t["ms_per_step"], 1)}
        if ref is not None and b != "py":
            d0 = compare(ref, results[b])
            dm, dp = compare(ref, results[b], -1), compare(ref, results[b], +1)
            row.update({"max_dF2_deg": d0[0], "max_dstate_rel": d0[1],
                        "shifted_dstate_rel": min(dm[1], dp[1])})
            if b in ("fixed", "native"):
                row["verdict"] = "reference only"
            else:
                lag = min(dm[1], dp[1]) < 0.1 * d0[1] and d0[1] > TOL_STATE
                ok = d0[0] < TOL_F2 and d0[1] < TOL_STATE and not lag
                row["verdict"] = "PASS" if ok else ("FAIL (one-step lag)" if lag else "FAIL")
                failed |= not ok
        else:
            row["verdict"] = "reference" if b == "py" else "-"
        rows.append(row)

    base = timings.get("fixed", {}).get("ms_per_step")
    lines = [f"# WingLoop 3.0 backend cross-check: {build} aircraft", "",
             f"Test aircraft (aswing_geometry_{build}), 1-cosine gust, N = {a.N}, Dt = {a.Dt} s, K = 1, "
             f"ASWING `{exe}`.",
             "Differences against the Python run. Timings: one run each, this machine.", "",
             "| backend | verdict | max dF2 [deg] | max dstate / range | shifted by 1 step | ms/step | "
             "vs fixed stick | overhead [s] | status |",
             "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        f = lambda k, fmt="{:.2e}": fmt.format(r[k]) if k in r and r[k] is not None and np.isfinite(r[k]) else "-"
        rel = f"{(r['ms_per_step'] / base - 1) * 100:+.1f} %" if base and "ms_per_step" in r else "-"
        lines.append(f"| {r['backend']} | {r.get('verdict', '-')} | {f('max_dF2_deg')} | {f('max_dstate_rel')} | "
                     f"{f('shifted_dstate_rel')} | {r.get('ms_per_step', '-')} | {rel} | {r.get('overhead_s', '-')} | "
                     f"{r.get('status', '-')} |")
    report = "\n".join(lines) + "\n"
    with open(os.path.join(a.out, "report.md"), "w") as fh:
        fh.write(report)
    with open(os.path.join(a.out, "report.json"), "w") as fh:
        json.dump(rows, fh, indent=2, default=float)
    print("\n" + report)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
        for b, r in results.items():
            ax[0].plot(r["t"], r["F2"], label=b, lw=1.2)
            ax[1].plot(r["t"], r["Elev"], label=b, lw=1.2)
        ax[0].set_ylabel("F2 [deg]"); ax[1].set_ylabel("pitch [deg]"); ax[1].set_xlabel("t [s]")
        ax[0].legend(ncol=4, fontsize=8)
        for x in ax:
            x.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, "comparison.png"), dpi=130)
    except Exception as exc:
        print("plot skipped:", exc)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
