# =============================================================================
# WingLoop — make_murua_validation.py (thesis figures fig:validation_1mcosine and its error figure)
# =============================================================================
# Copyright (c) 2026 Leonardo Avoni (avonileonardo@gmail.com)
# Part of WingLoop (CC BY-NC-SA 4.0).
# =============================================================================
"""
Murua_validation.pdf and Murua_validation_error.pdf (vector PDF), in the style of the thesis
figures (matplotlib defaults, 12 x 9.14 in, same colours, markers, line styles, labels, grey gust
band), from the four validation runs: fixed stick, ASWING's own PID (native), WingLoop Python
K = 1 (py, v1_k1 gains) and K = 10 (py_k10, v1_k10 gains). The other backends (MATLAB, Simulink,
FMU, Simulink V2) give the same result as py and are not plotted.

    python make_murua_validation.py RUNS_DIR [--out DIR]

RUNS_DIR holds one folder per run with its result.npz: fixed, native, py, py_k10, or the
benchmark's r1_fixed, r1_native, r1_py, r1_py_k10 (benchmark.py calls make() at the end).

Quantities (validation_check.extract): pitch = Elev. [deg]; elevator = F2 [deg]; pitch integral
= ASWING's integral of (theta - theta_c) dt (global state variable 54, stored in rad s, plotted
in deg s; it moves only when ASWING's own control law runs, so it is zero in the WingLoop and
fixed-stick runs); WRBM = moment component 1 of the wing node at y = 0 [N m]; tip deflection =
mean z of the two wing tips [m]. Error figure: WingLoop K = 1 minus ASWING control.
"""

import os
import argparse

import numpy as np

from validation_check import find_runs

FIGSIZE = (12.0, 9.14)          # in: 864 x 658 pt, the size of the thesis figures
GUST = (0.5, 1.5)               # s: the wing is inside the gust (15 m ahead, 30 m long, at 30 m/s)
BAND = dict(color="k", alpha=0.14, lw=0, zorder=0)
MARKEVERY = 20                  # one marker every 0.2 s at Dt = 0.01 s
RUNS = [("fixed", "ASWING fixed-stick", dict(color="C0", marker="s", markevery=MARKEVERY)),
        ("native", "ASWING control", dict(color="C1", marker="x", markevery=MARKEVERY)),
        ("py", "WingLoop K=1", dict(color="C2")),
        ("py_k10", "WingLoop K=10", dict(color="C3", ls="--"))]
PANELS = [("theta", r"$\theta$ [°]"), ("F2", "Elevator [°]"), ("int", r"$\int(\theta-\theta_c)dt$ [°s]"),
          ("wrbm", "WRBM [Nm]"), ("tip", "Tip Deflection [m]")]


def make(runs_dir, out=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update(matplotlib.rcParamsDefault)
    plt.rcParams["pdf.fonttype"] = 42                           # embedded fonts, all vector
    out = out or runs_dir
    R = find_runs(runs_dir)
    missing = [k for k, _, _ in RUNS if k not in R]
    if missing:
        print(f"[make_murua_validation] missing runs {missing} in {runs_dir}")

    fig, ax = plt.subplots(len(PANELS), 1, figsize=FIGSIZE, sharex=True)
    for a in ax:
        a.axvspan(*GUST, **BAND)
        a.grid(True)
    for key, label, style in RUNS:
        if key in R:
            for a, (q, _) in zip(ax, PANELS):
                a.plot(R[key]["t"], R[key][q], label=label, **style)
    for a, (_, yl) in zip(ax, PANELS):
        a.set_ylabel(yl)
    ax[0].legend(loc="upper right")
    ax[-1].set_xlabel("Time (t)")
    fig.tight_layout()
    p1 = os.path.join(out, "Murua_validation.pdf")
    fig.savefig(p1)
    plt.close(fig)

    p2 = None
    if "py" in R and "native" in R:
        fig, ax = plt.subplots(len(PANELS), 1, figsize=FIGSIZE, sharex=True)
        for a, (q, yl) in zip(ax, PANELS):
            a.axvspan(*GUST, **BAND)
            a.plot(R["py"]["t"], R["py"][q] - R["native"][q])
            a.set_ylabel("Error in " + yl)
            a.grid(True)
        ax[-1].set_xlabel("Time (t)")
        fig.tight_layout()
        p2 = os.path.join(out, "Murua_validation_error.pdf")
        fig.savefig(p2)
        plt.close(fig)
        for q, _ in PANELS:
            print(f"[make_murua_validation] max |K=1 - ASWING control| {q}: "
                  f"{np.abs(R['py'][q] - R['native'][q]).max():.4g}")
    print(f"[make_murua_validation] wrote {p1}" + (f" and {p2}" if p2 else ""))
    return p1, p2


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs_dir")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    make(a.runs_dir, a.out)
