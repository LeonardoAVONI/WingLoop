# =============================================================================
# WingLoop — plot_backends.py (time histories of every controller backend)
# =============================================================================
# Copyright (c) 2026 Leonardo Avoni (avonileonardo@gmail.com)
# Part of WingLoop (CC BY-NC-SA 4.0).
# =============================================================================
"""
Plot the main quantities of every backend's run on the same figure.

    python plot_backends.py [--results results_v3] [--build stable|modified|all]

Reads <results>/<build>/<backend>/result.npz (written by run_all_backends.py) and writes
<results>/<build>/backends_timeseries.png: elevator F2, engines E1, pitch, angle of attack,
altitude change, airspeed, and the difference of each backend from the Python run (pitch and
F2, log scale; an exact match is drawn at the bottom of the axis). Backends that give the same
result overlap, so they are drawn with decreasing widths and different dash patterns.
"""

import os
import argparse

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ORDER = ["py", "mat", "sim", "fmu_rebuild", "fmu", "simv2", "native", "fixed"]
LABEL = {"py": "Python", "mat": "MATLAB", "sim": "Simulink V1", "fmu_rebuild": "FMU (re-exported)",
         "fmu": "FMU", "simv2": "Simulink V2", "native": "ASWING control law (.con)",
         "fixed": "fixed stick (no control)"}
STYLE = {  # (color, width, dashes): controller backends overlap when identical
    "py": ("#1f77b4", 5.0, ()), "mat": ("#ff7f0e", 3.8, (6, 2)), "sim": ("#2ca02c", 2.8, (3, 2)),
    "fmu_rebuild": ("#d62728", 2.2, (8, 3, 2, 3)), "fmu": ("#9467bd", 1.6, (1, 1.5)),
    "simv2": ("#000000", 1.0, ()), "native": ("#8c564b", 1.6, (10, 2)), "fixed": ("#7f7f7f", 1.4, (2, 2)),
}
FLOOR = 1e-12      # where an exact match (difference 0) is drawn on the log axes


def load(path):
    from WingLoop_Library import import_data_dict
    V = import_data_dict(path)["ModelVariables"]
    g = lambda k: np.asarray(V[k]["values"], dtype=float)
    return {"t": g("Time"), "F2": g("F2"), "E1": g("E1"), "pitch": g("Elev."), "alpha": g("Alpha"),
            "Z": g("earth Z") - g("earth Z")[0], "V": g("Velocity")}


def plot_build(results, build):
    folder = os.path.join(results, build)
    runs = {b: load(os.path.join(folder, b, "result.npz")) for b in ORDER
            if os.path.exists(os.path.join(folder, b, "result.npz"))}
    if not runs:
        print(f"[plot_backends] no results in {folder}")
        return None
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    panels = [("F2", "elevator F2 [deg]"), ("E1", "engine E1 (= E2)"), ("pitch", "pitch [deg]"),
              ("alpha", "angle of attack [deg]"), ("Z", "altitude change [m]"), ("V", "airspeed [m/s]")]
    fig, axes = plt.subplots(4, 2, figsize=(14, 13), sharex=True)
    for ax, (key, ylabel) in zip(axes.flat[:6], panels):
        for b, r in runs.items():
            c, w, d = STYLE[b]
            ax.plot(r["t"], r[key], color=c, lw=w, dashes=d, label=LABEL[b])
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)

    ref = runs.get("py")
    for ax, key, ylabel in ((axes.flat[6], "pitch", "|pitch − Python| [deg]"),
                            (axes.flat[7], "F2", "|F2 − Python| [deg]")):
        if ref is not None:
            for b, r in runs.items():
                if b == "py":
                    continue
                n = min(len(r[key]), len(ref[key]))
                diff = np.maximum(np.abs(r[key][:n] - ref[key][:n]), FLOOR)
                c, w, d = STYLE[b]
                ax.semilogy(r["t"][:n], diff, color=c, lw=w, dashes=d, label=LABEL[b])
            ax.axhline(FLOOR, color="k", lw=0.5)
            ax.text(0.01, FLOOR * 3, "identical to Python", transform=ax.get_yaxis_transform(), fontsize=8)
        ax.set_ylabel(ylabel)
        ax.set_ylim(FLOOR / 3, 100)
        ax.grid(alpha=0.3, which="both")
    for ax in axes[-1]:
        ax.set_xlabel("time [s]")
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.suptitle(f"WingLoop 3.0, test aircraft through a 1-cosine gust: {build} build, every backend",
                 y=0.995, fontsize=12)
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.975), ncol=4, fontsize=9,
               frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out = os.path.join(folder, "backends_timeseries.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"[plot_backends] {out}")
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", default=os.path.join(HERE, "results_v3"))
    ap.add_argument("--build", default="all", choices=["stable", "modified", "all"])
    a = ap.parse_args()
    for build in (["stable", "modified"] if a.build == "all" else [a.build]):
        plot_build(a.results, build)
