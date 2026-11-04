# =============================================================================
# WingLoop — validation_check.py (time histories of the test case, against a reference machine)
# =============================================================================
# Copyright (c) 2026 Leonardo Avoni (avonileonardo@gmail.com)
# Part of WingLoop (CC BY-NC-SA 4.0).
# =============================================================================
"""
Time histories of the four validation runs of the thesis (fixed stick, ASWING's own PID, WingLoop
Python K = 1, WingLoop Python K = 10), in the layout of the thesis figure fig:validation_1mcosine
(pitch, elevator, ASWING's pitch integral, WRBM, tip deflection; grey band = wing inside the gust),
with the reference runs of another machine overlaid, and the error WingLoop K = 1 - ASWING control.

    python validation_check.py RUNS_DIR [--reference reference_7550_validation.npz] [--out DIR]
    python validation_check.py RUNS_DIR --make-reference reference_7550_validation.npz

RUNS_DIR holds one folder per run with its result.npz: fixed, native, py, py_k10 (or the
benchmark's r1_fixed, r1_native, r1_py, r1_py_k10, found automatically). benchmark.py calls
check() at the end. Writes validation_check.pdf, validation_check_error.pdf and
validation_check.md (largest difference to the reference per run and channel).

Quantities: pitch = Elev. [deg]; elevator = F2 [deg]; integral = ASWING's integral of
(theta - theta_c) dt (global variable 54 of the state block, rad s, shown in deg s; zero when
ASWING runs fixed stick, i.e. in every WingLoop run); WRBM = moment component 1 of the wing node
at y = 0 [N m]; tip deflection = mean z of the two wing tips [m] (absolute, as in the thesis figure).
"""

import os
import sys
import argparse

import numpy as np

RUNS = [("fixed", "ASWING fixed-stick", dict(color="C0", marker="s")),
        ("native", "ASWING control", dict(color="C1", marker="x")),
        ("py", "WingLoop K=1", dict(color="C2")),
        ("py_k10", "WingLoop K=10", dict(color="C3", ls="--"))]
PANELS = [("theta", r"$\theta$ [°]"), ("F2", "Elevator [°]"), ("int", r"$\int(\theta-\theta_c)dt$ [°s]"),
          ("wrbm", "WRBM [Nm]"), ("tip", "Tip Deflection [m]")]
N_WING = 48                       # main-wing structural nodes (default mesh): first rows of the states
K_INT_THETA = 54                  # global variable index of ASWING's integral of (theta - theta_c) dt
GUST = (0.5, 1.5)                 # s: the wing is inside the gust (15 m ahead, 30 m long, at 30 m/s)


def extract(result_npz):
    from WingLoop_Library import import_data_dict
    d = import_data_dict(result_npz)
    V, P = d["ModelVariables"], d["ModelParameters"]
    S = np.asarray(d["ModelStates"], float)
    iitot = P["IITOT"]
    Q = S[:, :18 * iitot].reshape(len(S), iitot, 18)[:, :N_WING]
    root = int(np.argmin(np.abs(Q[0, :, 1])))
    tips = [int(np.argmin(Q[0, :, 1])), int(np.argmax(Q[0, :, 1]))]
    k = 18 * iitot + 12 * P["NJOIN"] + P["NNTOT"] + K_INT_THETA
    return {"t": np.asarray(V["Time"]["values"], float), "theta": np.asarray(V["Elev."]["values"], float),
            "F2": np.asarray(V["F2"]["values"], float), "int": np.degrees(S[:, k]),
            "wrbm": Q[:, root, 6], "tip": Q[:, tips, 2].mean(axis=1)}


def find_runs(runs_dir):
    out = {}
    for key, _, _ in RUNS:
        for name in (key, f"r1_{key}"):
            p = os.path.join(runs_dir, name, "result.npz")
            if os.path.exists(p):
                out[key] = extract(p)
                break
    return out


def check(runs_dir, reference=None, out=None):
    """Plots and a difference report; returns {run: {channel: max |difference|}} (None without reference)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out = out or runs_dir
    R = find_runs(runs_dir)
    ref = None
    if reference and os.path.exists(reference):
        z = np.load(reference)
        ref = {k: {q: z[f"{k}_{q}"] for q in ["t"] + [p for p, _ in PANELS]} for k, _, _ in RUNS if f"{k}_t" in z}
    fig, ax = plt.subplots(len(PANELS), 1, figsize=(10, 8), sharex=True)
    for a in ax:
        a.axvspan(*GUST, color="0.88", lw=0, zorder=0)
        a.grid(True)
    for key, label, style in RUNS:
        if key not in R:
            continue
        r = R[key]
        for a, (q, _) in zip(ax, PANELS):
            a.plot(r["t"], r[q], label=label, markevery=20, **style)
            if ref and key in ref:
                a.plot(ref[key]["t"], ref[key][q], color="k", lw=0.6, ls=":",
                       label="reference" if (key == "fixed" and q == "theta") else None)
    for a, (_, yl) in zip(ax, PANELS):
        a.set_ylabel(yl)
    ax[0].legend(loc="upper right", fontsize=8)
    ax[-1].set_xlabel("Time (t)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "validation_check.pdf"))
    plt.close(fig)
    if "py" in R and "native" in R:
        fig, ax = plt.subplots(len(PANELS), 1, figsize=(10, 8), sharex=True)
        for a, (q, yl) in zip(ax, PANELS):
            a.axvspan(*GUST, color="0.88", lw=0, zorder=0)
            a.plot(R["py"]["t"], R["py"][q] - R["native"][q])
            if ref and "py" in ref and "native" in ref:
                a.plot(ref["py"]["t"], ref["py"][q] - ref["native"][q], color="k", lw=0.6, ls=":")
            a.set_ylabel("Error in " + yl, fontsize=8)
            a.grid(True)
        ax[-1].set_xlabel("Time (t)")
        fig.tight_layout()
        fig.savefig(os.path.join(out, "validation_check_error.pdf"))
        plt.close(fig)
    lines = ["# Validation runs against the reference", "",
             f"Runs: {runs_dir}. Reference: {reference if ref else 'none'}.", ""]
    diffs = None
    if ref:
        diffs = {}
        lines += ["| run | " + " | ".join(q for q, _ in PANELS) + " |", "|---" * (len(PANELS) + 1) + "|"]
        for key, _, _ in RUNS:
            if key in R and key in ref and len(R[key]["t"]) == len(ref[key]["t"]):
                diffs[key] = {q: float(np.abs(R[key][q] - ref[key][q]).max()) for q, _ in PANELS}
                lines.append(f"| {key} | " + " | ".join(f"{diffs[key][q]:.3g}" for q, _ in PANELS) + " |")
        lines += ["", "Largest absolute difference over the run (deg, deg, deg s, N m, m). Round-off between "
                  "machines gives about 1e-4 or less; anything larger means a different ASWING build, files or "
                  "settings."]
    for key, _, _ in RUNS:
        if key in R:
            r = R[key]
            lines.append(f"- {key}: trim theta {r['theta'][0]:.4f} deg, F2 {r['F2'][0]:.4f} deg; pitch max "
                         f"{r['theta'].max():.4f} deg at {r['t'][r['theta'].argmax()]:.2f} s; F2 min "
                         f"{r['F2'].min():.4f} deg at {r['t'][r['F2'].argmin()]:.2f} s")
    with open(os.path.join(out, "validation_check.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    return diffs


def make_reference(runs_dir, path):
    R = find_runs(runs_dir)
    np.savez_compressed(path, **{f"{k}_{q}": v for k, r in R.items() for q, v in r.items()})
    print(f"reference with runs {sorted(R)} written to {path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs_dir")
    ap.add_argument("--reference", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                         "reference_7550_validation.npz"))
    ap.add_argument("--make-reference", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.make_reference:
        make_reference(a.runs_dir, a.make_reference)
    else:
        check(a.runs_dir, a.reference, a.out)
