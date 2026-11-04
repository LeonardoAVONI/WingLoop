# =============================================================================
# WingLoop — benchmark.py (wall-time benchmark of the controller backends)
# =============================================================================
# Copyright (c) 2026 Leonardo Avoni (avonileonardo@gmail.com)
# Part of WingLoop (CC BY-NC-SA 4.0).
# =============================================================================
"""
Wall-time benchmark of WingLoop on the test aircraft (Murua HALE, 30 m/s, 20 km density,
1-cosine gust 15 m ahead, H = 15 m, 3 m/s), for the thesis "Time Performance" section.

    python benchmark.py [--scenario v1_k1] [--repeats 5] [--build stable] [--out DIR]
                        [--configs a,b,...] [--no-plot-runs] [--force] [--quick]

Configurations (label: what runs):
    aswing_fixed    ASWING alone, fixed stick: one 300-step march, no per-step I/O
    aswing_native   ASWING alone, its own PID (.con with the scenario's gains)
    fixed           WingLoop, fixed stick (per-step point file, no controller)
    native          WingLoop stepping, ASWING's own PID
    py, py_k10      WingLoop Python controller (scenario gains; K = 10 with v1_k10 gains)
    mat             MATLAB controller
    sim             Simulink, model restarted at every step (WingLoop 2.0 method)
    fmu             Simulink exported as an FMU (exported once, before the repeats)
    simv2           Simulink, persistent session (WingLoop 3.0 method)
Every WingLoop configuration runs with the live plot off and on (except with --no-plot-runs).
Each repeat runs all configurations in a new random order (drift and throttling average out);
each run is its own process on its own copy of aswing_geometry_<build>/.

Before the runs: machine checks (power profile, AC power, load), environment record, and a
re-trim of the aircraft with this machine's ASWING build, which must give the reference trim
(else the run stops: a different build would change the case).

Output in DIR (default results_bench/<host>_<date>/): environment.json, runs.csv (one row
per run: overhead, simulation time, ms/step and its split), summary.md (mean and std of overhead
and computation per configuration), timing_bars.pdf (overhead + computation + live-plot time of
the thesis configurations; redraw with --replot DIR), the result files of the first repeat, and validation_check.pdf,
validation_check_error.pdf, validation_check.md: the time histories of the fixed, native, py and
py_k10 runs in the layout of the thesis validation figure, with the 7550 reference runs
(reference_7550_validation.npz) overlaid and their largest differences listed; and the thesis
figures Murua_validation.pdf, Murua_validation_error.pdf (make_murua_validation.py: fixed stick,
ASWING control, WingLoop K = 1 and K = 10 only, in the thesis style).
Copy DIR back to the main machine.
"""

import os
import sys
import csv
import json
import time
import random
import shutil
import socket
import hashlib
import argparse
import platform
import subprocess

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import wingloop_testrun as wt                                      # noqa: E402

ALIASES = {"stable": "aswing_stable", "modified": "aswing_modified"}
REF_TRIM = {"stable": {"theta_deg": 4.6919, "F2ref": -4.639, "E1": 2.746}}   # make_trim.py, 2026-11-04
TRIM_TOL = {"theta_deg": 2e-3, "F2ref": 2e-3, "E1": 2e-3}
WINGLOOP_CONFIGS = {        # label: (backend, scenario override or None)
    "fixed": ("fixed", None), "native": ("native", None), "py": ("py", None), "py_k10": ("py", "v1_k10"),
    "mat": ("mat", None), "sim": ("sim", None), "fmu": ("fmu", None), "simv2": ("simv2", None)}
RAW_CONFIGS = ("aswing_fixed", "aswing_native")
ALL = list(RAW_CONFIGS) + list(WINGLOOP_CONFIGS)


# ----------------------------------------------------------------------------- environment
def _sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=30).stdout.strip()
    except Exception as exc:                                      # noqa: BLE001
        return f"unavailable ({exc})"


def machine_checks():
    """Return (environment dict, list of warnings)."""
    env = {"host": socket.gethostname(), "date": time.strftime("%Y-%m-%d %H:%M:%S"),
           "cpu": _sh("lscpu | grep 'Model name' | sed 's/.*: *//'"),
           "cpus": os.cpu_count(), "ram": _sh("free -g | awk '/Mem:/{print $2\" GB\"}'"),
           "os": platform.platform(), "python": platform.python_version(),
           "power_profile": _sh("powerprofilesctl get 2>/dev/null || cat /sys/firmware/acpi/platform_profile"),
           "ac_online": _sh("cat /sys/class/power_supply/A*/online 2>/dev/null | head -1"),
           "loadavg_1min": os.getloadavg()[0],
           "wingloop_git": _sh(f"git -C {HERE} rev-parse --short HEAD"),
           "wingloop_dirty": bool(_sh(f"git -C {HERE} status --porcelain --untracked-files=no"))}
    warn = []
    if "performance" not in env["power_profile"]:
        warn.append(f"power profile is '{env['power_profile']}', not performance")
    if env["ac_online"] not in ("1", ""):
        warn.append("not on AC power")
    if env["loadavg_1min"] > 1.0:
        warn.append(f"1-min load average {env['loadavg_1min']:.2f} > 1: close other programs")
    return env, warn


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def check_trim(exe, build, out):
    """Re-trim with this machine's ASWING into out/trim_check and compare with REF_TRIM."""
    d = os.path.join(out, "trim_check")
    r = subprocess.run([sys.executable, os.path.join(HERE, "make_trim.py"), build, "--aswing", exe, "--out", d],
                       cwd=HERE, capture_output=True, text=True)
    path = os.path.join(d, "trim_reference.json")
    if r.returncode != 0 or not os.path.exists(path):
        raise RuntimeError(f"make_trim.py failed:\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")
    trim = json.load(open(path))
    ref = REF_TRIM.get(build)
    bad = {k: (trim[k], v) for k, v in (ref or {}).items() if abs(trim[k] - v) > TRIM_TOL[k]}
    return trim, bad


# ----------------------------------------------------------------------------- runs
def run_wingloop(label, backend, scenario, plot, exe, build, N, case, tj):
    cmd = [sys.executable, os.path.join(HERE, "wingloop_testrun.py"), backend, "--build", build, "--aswing", exe,
           "--N", str(N), "--sim-directory", case, "--out", "result", "--timings-json", tj, "--scenario", scenario]
    if plot:
        cmd.append("--plot")
    with open(tj.replace(".json", ".log"), "w") as log:
        r = subprocess.run(cmd, cwd=HERE, stdout=log, stderr=subprocess.STDOUT)
    if r.returncode != 0 or not os.path.exists(tj):
        return {"status": f"FAILED (see {os.path.basename(tj).replace('.json', '.log')})"}
    return json.load(open(tj))


def run_aswing_alone(native, scenario, exe, N, Dt, case):
    """ASWING alone: same files and OPER setup as WingLoop, then one N-step march (no per-step
    point file, no controller exchange). Returns timings."""
    from WingLoop_Library.Aswing_Director import Aswing_Director
    t_first = time.time()
    files = list(wt.CASE_FILES)
    if native:
        files.append(wt.make_con(case, scenario))
    asw = Aswing_Director(aswing_path=exe, write_timeout=1200)
    asw.start_aswing(directory=case, filename=wt.GEOMETRY.replace(".asw", ""), print_output=False)
    for c in ("PLPA", "G", ""):
        asw.send_command_and_receive(c)
    for f in files:
        asw.Load_Files_ASW(f)
    asw.send_command_and_receive("oper")
    if native:
        asw.send_command_and_receive("%", expect=r"\( ABCDFLPQSX\.\+- \): $")
        for k in ("f", "c", "l"):
            asw.send_command_and_receive(k, expect=r"\( ABCDFLPQSX\.\+- \): $")
        asw.send_command_and_receive("d")
    out, _ = asw.send_command_and_receive(".")
    if ".OPERs" in out[-200:]:
        asw.send_command_and_receive(".")
    t_start = time.time()
    asw.march(Dt, N, control_file=None, append=False)
    t_end = time.time()
    asw.write_point_file("final_point")
    txt = open(os.path.join(case, "final_point")).read()
    asw.quit_and_close_aswing()
    os.chdir(HERE)
    sim = t_end - t_start
    theta = float(txt.split("Elev.  :")[1].split()[0]) if "Elev.  :" in txt else float("nan")
    return {"status": "completed" if "Status: Converged" in txt else "final point not converged",
            "overhead_s": t_start - t_first, "simulation_s": sim, "ms_per_step": sim / N * 1000,
            "aswing_march_s": sim, "read_point_s": 0.0, "controller_s": 0.0, "plot_s": 0.0, "other_s": 0.0,
            "final_theta_deg": theta, "plot": False}


# ----------------------------------------------------------------------------- report
FIELDS = ["repeat", "order", "config", "plot", "scenario", "K", "status", "overhead_s", "simulation_s", "ms_per_step",
          "aswing_march_s", "read_point_s", "controller_s", "plot_s", "other_s", "newton_restarts", "tsplits"]


def summarize(rows, out, env, args, fmu_export):
    groups = {}
    for r in rows:
        if str(r.get("status", "")).startswith("completed"):
            groups.setdefault((r["config"], r["plot"]), []).append(r)
    base = groups.get(("aswing_native", False)) or groups.get(("aswing_fixed", False))
    base_sim = np.mean([g["simulation_s"] for g in base]) if base else float("nan")
    lines = [f"# WingLoop benchmark, {env['host']} ({env['cpu']}), {env['date']}", "",
             f"Scenario {args.scenario}, build {args.build}, N = {args.N}, Dt = {args.Dt} s, {args.repeats} repeats "
             f"in random order. Power profile {env['power_profile']}, AC {env['ac_online']}. "
             f"WingLoop {env['wingloop_git']}{' (modified files)' if env['wingloop_dirty'] else ''}.",
             f"Reference for the increase: ASWING alone with its own controller, {base_sim:.2f} s.",
             f"FMU export (one-off, before the repeats): {fmu_export:.1f} s." if fmu_export else "", "",
             "| config | plot | n | overhead [s] | computation [s] | increase | ms/step |", "|---|---|---|---|---|---|---|"]
    order = {c: i for i, c in enumerate(ALL)}
    table = []
    for (cfg, plot), g in sorted(groups.items(), key=lambda kv: (order.get(kv[0][0], 99), kv[0][1])):
        m = lambda k: (np.mean([x[k] for x in g]), np.std([x[k] for x in g]))
        sim, ov = m("simulation_s"), m("overhead_s")
        parts = [np.mean([x[k] for x in g]) / args.N * 1000 for k in
                 ("aswing_march_s", "read_point_s", "controller_s", "plot_s", "other_s")]
        inc = (sim[0] / base_sim - 1) * 100 if np.isfinite(base_sim) else float("nan")
        lines.append(f"| {cfg} | {'on' if plot else 'off'} | {len(g)} | {ov[0]:.2f} ± {ov[1]:.2f} | "
                     f"{sim[0]:.2f} ± {sim[1]:.2f} | {inc:+.1f} % | {sim[0] / args.N * 1000:.1f} |")
        table.append({"config": cfg, "plot": plot, "n": len(g), "overhead_mean": ov[0], "overhead_std": ov[1],
                      "sim_mean": sim[0], "sim_std": sim[1], "increase_pct": inc,
                      "split_ms_per_step": dict(zip(["aswing", "point_io", "controller", "plot", "other"], parts))})
    lines += ["", "Overhead: start-up before the first step. Computation: the time-marching loop (increase and "
              "ms/step refer to it). The per-step split of the loop is kept in runs.csv and summary.json."]
    with open(os.path.join(out, "summary.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(table, f, indent=2, default=float)
    print("\n".join(lines))
    try:
        bar_plot(table, os.path.join(out, "timing_bars.pdf"))
    except Exception as exc:                                       # noqa: BLE001
        print("bar plot skipped:", exc)


# thesis timing figure: (config, label); Simulink V1 ("sim") is left out and discussed in the text
BARS = [("aswing_fixed", "ASWING fixed-stick"), ("aswing_native", "ASWING control"), ("py", "WL Python"),
        ("py_k10", "WL Python (K=10)"), ("mat", "WL MATLAB"), ("fmu", "WL Simulink FMU"),
        ("simv2", "WL Simulink Mk2")]


def bar_plot(table, path):
    """Wall time of one run (N steps), stacked: overhead (start-up, before the first step) +
    computation (the time-marching loop, live plot off) + the extra computation time with the live
    plot on (plot-on minus plot-off mean, WingLoop configurations only). Error bars: std of the
    computation time over the repeats (plot off, then plot on)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update(matplotlib.rcParamsDefault)
    plt.rcParams["pdf.fonttype"] = 42
    T = {(r["config"], bool(r["plot"])): r for r in table}
    bars = [(c, lab) for c, lab in BARS if (c, False) in T]
    y = np.arange(len(bars))
    ov = np.array([T[c, False]["overhead_mean"] for c, _ in bars])
    comp = np.array([T[c, False]["sim_mean"] for c, _ in bars])
    comp_sd = np.array([T[c, False]["sim_std"] for c, _ in bars])
    on = [T.get((c, True)) for c, _ in bars]
    live = np.array([max(r["sim_mean"] - cm, 0.0) if r else 0.0 for r, cm in zip(on, comp)])
    live_sd = np.array([r["sim_std"] if r else 0.0 for r in on])
    fig, ax = plt.subplots(figsize=(12.0, 0.55 * len(bars) + 1.4))
    ax.barh(y, ov, color="C7", label="overhead")
    ax.barh(y, comp, left=ov, color="C0", label="computation", xerr=comp_sd, capsize=3)
    has = live > 0
    ax.barh(y[has], live[has], left=(ov + comp)[has], color="C1", alpha=0.6, hatch="//",
            label="live plot (added)", xerr=live_sd[has], capsize=3)
    for k in range(len(bars)):
        tot = ov[k] + comp[k] + live[k]
        txt = f"{comp[k]:.2f} s" + (f" + {live[k]:.2f} s" if has[k] else "") + f"  (overhead {ov[k]:.2f} s)"
        ax.text(tot + 0.01 * (ov + comp + live).max() + (live_sd[k] if has[k] else comp_sd[k]), y[k], txt,
                va="center", fontsize=9)
    ax.set_yticks(y, [lab for _, lab in bars])
    ax.invert_yaxis()
    ax.set_xlim(0, 1.3 * (ov + comp + live).max())
    ax.set_xlabel("Wall time of the run [s]")
    ax.grid(True, axis="x")
    ax.set_axisbelow(True)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    print(f"[benchmark] wrote {path}")


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="v1_k1", choices=sorted(wt.SCENARIOS))
    ap.add_argument("--build", default="stable", choices=sorted(ALIASES))
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--N", type=int, default=300)
    ap.add_argument("--Dt", type=float, default=0.01)
    ap.add_argument("--configs", default=",".join(ALL), help="subset of: " + ",".join(ALL))
    ap.add_argument("--no-plot-runs", action="store_true", help="only the plot-off runs")
    ap.add_argument("--aswing", default=None, help="ASWING executable or folder (default: the build's alias)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--force", action="store_true", help="run despite machine-check warnings or a trim mismatch")
    ap.add_argument("--quick", action="store_true", help="1 repeat, N = 50: a smoke test of the script")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--replot", metavar="DIR", default=None,
                    help="only redraw DIR/timing_bars.pdf from DIR/summary.json (no runs)")
    a = ap.parse_args()
    if a.replot:
        bar_plot(json.load(open(os.path.join(a.replot, "summary.json"))), os.path.join(a.replot, "timing_bars.pdf"))
        return
    if a.quick:
        a.repeats, a.N = 1, 50
    configs = [c.strip() for c in a.configs.split(",") if c.strip()]
    unknown = set(configs) - set(ALL)
    if unknown:
        ap.error(f"unknown configs {sorted(unknown)}")

    from WingLoop_Library.Aswing_Director import resolve_aswing_executable
    exe = resolve_aswing_executable(a.aswing, None if a.aswing else ALIASES[a.build])
    env, warn = machine_checks()
    out = os.path.abspath(a.out or os.path.join(HERE, "results_bench", f"{env['host']}_{time.strftime('%Y%m%d_%H%M')}"))
    os.makedirs(out, exist_ok=True)
    env.update({"aswing": exe, "aswing_sha256": sha256(exe), "scenario": a.scenario, "build": a.build,
                "repeats": a.repeats, "N": a.N, "Dt": a.Dt, "configs": configs, "warnings": warn,
                "gains_deg_per_rad": wt.scenario_gains(a.scenario)})
    for w in warn:
        print("[benchmark] WARNING:", w)
    if warn and not a.force:
        sys.exit("[benchmark] fix the warnings above or pass --force")
    trim, bad = check_trim(exe, a.build, out)
    env["trim"] = trim
    print(f"[benchmark] trim on this machine: theta {trim['theta_deg']:.5f} deg, F2 {trim['F2ref']}, "
          f"E1 {trim['E1']} N")
    if bad:
        print(f"[benchmark] trim differs from the reference: {bad}")
        if not a.force:
            sys.exit("[benchmark] different ASWING build or files: stopping (pass --force to continue)")
    with open(os.path.join(out, "environment.json"), "w") as f:
        json.dump(env, f, indent=2, default=str)

    geometry = os.path.join(HERE, f"aswing_geometry_{a.build}")
    fresh = lambda case: (shutil.rmtree(case, ignore_errors=True),
                          shutil.copytree(geometry, case, ignore=shutil.ignore_patterns(
                              "input", "output", "result*", "*.npz", "*.pdf", "*.ps", "controller_config.json")))

    # one-off FMU export with this scenario's gains (the "fmu" runs then reuse it)
    fmu_export = None
    if "fmu" in configs:
        case = os.path.join(out, "fmu_export")
        fresh(case)
        print("[benchmark] exporting the FMU (fmu_rebuild) ...", flush=True)
        t = run_wingloop("fmu_export", "fmu_rebuild", a.scenario, False, exe, a.build, a.N, case,
                         os.path.join(out, "fmu_export.json"))
        fmu_export = t.get("overhead_s")
        if not str(t.get("status", "")).startswith("completed"):
            sys.exit(f"[benchmark] FMU export failed: {t.get('status')}")

    jobs = []
    for c in configs:
        for plot in ((False,) if c in RAW_CONFIGS or a.no_plot_runs else (False, True)):
            jobs.append((c, plot))
    rng = random.Random(a.seed)
    rows = []
    csv_path = os.path.join(out, "runs.csv")
    with open(csv_path, "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=FIELDS, extrasaction="ignore")
        wr.writeheader()
        for rep in range(1, a.repeats + 1):
            order = jobs[:]
            rng.shuffle(order)
            for i, (c, plot) in enumerate(order, 1):
                tag = f"r{rep}_{c}{'_plot' if plot else ''}"
                case = os.path.join(out, "runs", tag)
                fresh(case)
                print(f"[benchmark] repeat {rep}/{a.repeats}, {i}/{len(order)}: {c}{' (plot)' if plot else ''}",
                      flush=True)
                if c in RAW_CONFIGS:
                    t = run_aswing_alone(c == "aswing_native", a.scenario, exe, a.N, a.Dt, case)
                    t["K"] = 1
                else:
                    backend, sc = WINGLOOP_CONFIGS[c]
                    t = run_wingloop(c, backend, sc or a.scenario, plot, exe, a.build, a.N, case,
                                     os.path.join(out, "runs", tag + ".json"))
                row = {"repeat": rep, "order": i, "config": c, "plot": plot,
                       "scenario": (WINGLOOP_CONFIGS.get(c, (None, None))[1] or a.scenario), **t}
                rows.append(row)
                wr.writerow(row)
                fh.flush()
                print(f"           {t.get('status')}, simulation {t.get('simulation_s', float('nan')):.2f} s",
                      flush=True)
                if rep > 1:                                         # keep the result files of repeat 1 only
                    for f in os.listdir(case):
                        if f.startswith("result") or f.endswith((".state", ".pnt", ".set")):
                            os.remove(os.path.join(case, f))
    summarize(rows, out, env, a, fmu_export)
    # time histories of the first repeat against the 7550 reference runs (validation_check.py)
    try:
        import validation_check
        validation_check.check(os.path.join(out, "runs"), os.path.join(HERE, "reference_7550_validation.npz"), out)
    except Exception as exc:                                       # noqa: BLE001
        print("[benchmark] validation check skipped:", exc)
    # the thesis figures Murua_validation.pdf / _error.pdf from the same runs (make_murua_validation.py)
    try:
        import make_murua_validation
        make_murua_validation.make(os.path.join(out, "runs"), out)
    except Exception as exc:                                       # noqa: BLE001
        print("[benchmark] Murua validation figures skipped:", exc)
    print(f"\n[benchmark] done: {out}")


if __name__ == "__main__":
    main()
