"""
====================================================================================
PyControl_LivePlotter.py

Live plotting utility for ASWING simulation dictionaries produced by
PyControl_Text2Python.py  (read_aswing_file / initialize_data_dict).

Usage
-----
# --- Iterative (live) mode ---
plotter = ASWINGLivePlotter(
    parameter_list=["earth X", "earth Y", "earth Z", "Heading", "Elev.", "Bank"],
    total_sim_time=100.0        # seconds of simulation time (used to scale x-axis)
)

# Inside simulation loop:
for step in simulation_steps:
    data_dict = read_aswing_file(...)
    plotter.update(data_dict)

plotter.export("my_results.pdf")

# --- Post-processing (one-shot) mode ---
plotter2 = ASWINGLivePlotter(parameter_list=[...], total_sim_time=100.0)
plotter2.update(fully_populated_dict)      # identical call, works the same way
plotter2.export("my_results_post.pdf")
====================================================================================
"""

import time
import re
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.backends.backend_pdf import PdfPages


# ── helpers ──────────────────────────────────────────────────────────────────

def _detect_control_vars(model_variables: dict):
    """Return sorted lists of flap keys (F1…F20) and engine keys (E1…E20)."""
    flap_pat   = re.compile(r'^F(\d{1,2})$')
    engine_pat = re.compile(r'^E(\d{1,2})$')
    flaps   = sorted(
        [k for k in model_variables
         if flap_pat.match(k) and model_variables[k].get("values")],
        key=lambda x: int(x[1:]))
    engines = sorted(
        [k for k in model_variables
         if engine_pat.match(k) and model_variables[k].get("values")],
        key=lambda x: int(x[1:]))
    return flaps, engines


def _get_values(data_dict: dict, key: str):
    """Safe accessor – returns [] if key absent or has no values."""
    mv = data_dict.get("ModelVariables", {})
    return mv.get(key, {}).get("values", [])


def _get_unit(data_dict: dict, key: str):
    mv = data_dict.get("ModelVariables", {})
    return mv.get(key, {}).get("unit") or ""


def _compute_diverged_spans(t_arr: np.ndarray, conv_arr: np.ndarray):
    """
    Given parallel arrays of time values and boolean convergence flags,
    return a list of (t_start, t_end) intervals where the simulation was
    NOT converged.

    The span is extended half a time-step on each side so that a single
    unconverged point still produces a visible rectangle.
    """
    if len(t_arr) == 0 or len(conv_arr) == 0:
        return []

    n   = min(len(t_arr), len(conv_arr))
    t   = t_arr[:n]
    ok  = np.array(conv_arr[:n], dtype=bool)
    bad = ~ok

    if not bad.any():
        return []

    # Typical step size for half-step padding
    dt = float(np.median(np.diff(t))) * 0.5 if n > 1 else 0.0

    spans = []
    in_span = False
    t_start = None

    for i, is_bad in enumerate(bad):
        if is_bad and not in_span:
            t_start = t[i] - dt
            in_span = True
        elif not is_bad and in_span:
            spans.append((t_start, t[i - 1] + dt))
            in_span = False

    if in_span:                          # still diverged at the last sample
        spans.append((t_start, t[-1] + dt))

    return spans


# ── main class ────────────────────────────────────────────────────────────────

class ASWINGLivePlotter:
    """
    Live / post-processing plotter for ASWING simulation output dictionaries.

    Parameters
    ----------
    parameter_list : list[str]
        Variable names to show in the *State Variables* panel
        (e.g. ["earth X", "earth Y", "earth Z", "Heading", "Elev.", "Bank"]).
        Names must match keys in data_dict["ModelVariables"] (after rename_map
        is applied, i.e. use internal names).
    total_sim_time : float
        Expected total simulation time [s].  Used to pre-scale the x-axis so
        the live plot does not jump around.
    refresh_interval : float
        Minimum wall-clock seconds between figure refreshes (default 0.5 s).
        Set to 0 to redraw on every call to update().
    figsize : tuple
        Matplotlib figure size in inches.
    """

    # colour cycle for individual lines
    _COLOURS = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    def __init__(
        self,
        parameter_list: list,
        total_sim_time: float,
        refresh_interval: float = 0.5,
        figsize: tuple = (16, 10),
    ):
        self.parameter_list   = list(parameter_list)
        self.total_sim_time   = float(total_sim_time)
        self.refresh_interval = refresh_interval
        self.figsize          = figsize

        self._wall_t0        = time.time()
        self._last_draw_time = -np.inf

        # figure / axes created lazily on first update() call so that the
        # caller does not need a display at import time
        self._fig       = None
        self._info_text = None   # top-left Text artist, created in _build_figure
        self._axes  = {}   # name → Axes
        self._lines = {}  # name → Line2D

        # track whether we have set up axes for actuation channels yet
        self._actuation_keys: list = []

        # convergence overlay state
        self._div_spans: list = []       # list of axvspan patches, per axes key
        self._warn_text = None           # figure-level warning Text artist

    # ── public API ───────────────────────────────────────────────────────────

    def update(self, data_dict: dict, force_draw: bool = False):
        """
        Refresh the live figure with the latest contents of *data_dict*.

        May be called every simulation step (iterative mode) or just once
        after the simulation completes (post-processing mode).  The logic is
        identical either way.

        Parameters
        ----------
        data_dict   : dict  – the dictionary produced by read_aswing_file.
        force_draw  : bool  – if True, redraw regardless of refresh_interval.
        """
        # ── first call: build figure layout ──────────────────────────────
        if self._fig is None:
            self._build_figure(data_dict)

        # ── update header text (top-left annotation) ─────────────────────
        model_name = data_dict.get("ModelName", "N/A")
        time_vals  = _get_values(data_dict, "Time")
        sim_t      = time_vals[-1] if time_vals else 0.0
        wall_t     = time.time() - self._wall_t0

        self._info_text.set_text(
            f"Model:     {model_name}\n"
            f"Sim time:  {sim_t:.3f} s / {self.total_sim_time:.1f} s\n"
            f"Wall time: {wall_t:.1f} s"
        )

        # ── update every tracked series ───────────────────────────────────
        t_arr = np.asarray(time_vals) if time_vals else np.array([])

        all_keys = self.parameter_list + self._actuation_keys
        for key in all_keys:
            if key not in self._lines:
                continue
            vals = _get_values(data_dict, key)
            if not vals:
                continue
            v_arr = np.asarray(vals)
            n = min(len(t_arr), len(v_arr))
            if n == 0:
                continue
            self._lines[key].set_data(t_arr[:n], v_arr[:n])
            ax = self._axes[key]
            ax.relim()
            ax.autoscale_view(scalex=False)   # keep x fixed to total_sim_time

        # ── convergence overlays ──────────────────────────────────────────
        self._update_convergence_overlays(data_dict, t_arr)

        # ── throttle redraws ──────────────────────────────────────────────
        now = time.time()
        if force_draw or (now - self._last_draw_time) >= self.refresh_interval:
            self._fig.canvas.draw_idle()
            plt.pause(0.001)
            self._last_draw_time = now

    def export(self, filepath: str = "aswing_results.pdf"):
        """
        Save the current figure to *filepath* (PDF).

        Can be called after update() has been called at least once.
        If the figure does not exist yet, raises RuntimeError.
        """
        if self._fig is None:
            raise RuntimeError(
                "export() called before update(). "
                "Call update(data_dict) at least once first."
            )
        # Force a final full render
        self._fig.canvas.draw()
        with PdfPages(filepath) as pdf:
            pdf.savefig(self._fig, bbox_inches="tight")
        print(f"[ASWINGLivePlotter] Figure saved → {filepath}")

    def close(self):
        """Close the matplotlib figure window."""
        if self._fig is not None:
            plt.close(self._fig)
            self._fig = None

    # ── private helpers ───────────────────────────────────────────────────────

    def _update_convergence_overlays(self, data_dict: dict, t_arr: np.ndarray):
        """
        Recompute diverged time spans from IsConverged and redraw red axvspan
        patches on every subplot.  Old patches are removed before new ones are
        added so there is no accumulation across update() calls.
        """
        conv_vals = _get_values(data_dict, "IsConverged")
        if not conv_vals:
            return

        spans = _compute_diverged_spans(t_arr, np.array(conv_vals, dtype=bool))

        # ── remove previous patches ───────────────────────────────────────
        for patch in self._div_spans:
            try:
                patch.remove()
            except ValueError:
                pass   # already removed
        self._div_spans = []

        # ── draw new patches on every subplot ─────────────────────────────
        all_keys = self.parameter_list + self._actuation_keys
        for t0, t1 in spans:
            for key in all_keys:
                if key not in self._axes:
                    continue
                patch = self._axes[key].axvspan(
                    t0, t1,
                    facecolor="red", alpha=0.15,
                    zorder=0, label="_nolegend_"
                )
                self._div_spans.append(patch)

        # ── update the figure-level warning banner ────────────────────────
        if spans:
            # Build a compact summary, e.g. "⚠ Diverged: 35.0–40.0 s, 55.0–57.5 s"
            span_strs = ", ".join(f"{t0:.2g}–{t1:.2g} s" for t0, t1 in spans)
            self._warn_text.set_text(f"⚠  NOT CONVERGED: {span_strs}")
            self._warn_text.set_alpha(1.0)
        else:
            self._warn_text.set_text("")
            self._warn_text.set_alpha(0.0)

    def _build_figure(self, data_dict: dict):
        """
        Build the complete figure layout on first call.
        One subplot per tracked variable, grouped into two column-panels:
          Left  → user-requested parameter_list
          Right → detected actuation (flaps + engines)
        """
        matplotlib.use("TkAgg") if not matplotlib.is_interactive() else None

        flaps, engines = _detect_control_vars(data_dict.get("ModelVariables", {}))
        self._actuation_keys = flaps + engines

        n_left  = len(self.parameter_list)
        n_right = len(self._actuation_keys)
        n_rows  = max(n_left, n_right, 1)

        fig = plt.figure(figsize=self.figsize)
        model_name = data_dict.get("ModelName", "N/A")
        fig.canvas.manager.set_window_title(f"WingLoop: {model_name} live plot")
        fig.subplots_adjust(
            left=0.07, right=0.97,
            top=0.88, bottom=0.06,
            wspace=0.35, hspace=0.55
        )

        # Two GridSpec objects side-by-side (with a gap between them)
        gs_left  = gridspec.GridSpec(n_rows, 1, figure=fig,
                                     left=0.06, right=0.48,
                                     hspace=0.55)
        gs_right = gridspec.GridSpec(n_rows, 1, figure=fig,
                                     left=0.54, right=0.97,
                                     hspace=0.55)

        # ── left panel ────────────────────────────────────────────────────
        for i, key in enumerate(self.parameter_list):
            ax = fig.add_subplot(gs_left[i, 0])
            unit = _get_unit(data_dict, key)
            ylabel = f"{key} [{unit}]" if unit else key
            ax.set_ylabel(ylabel, fontsize=7)
            ax.set_xlabel("Time [s]", fontsize=7)
            ax.set_xlim(0, self.total_sim_time)
            ax.tick_params(labelsize=7)
            ax.grid(True, linestyle="--", alpha=0.5)

            colour = self._COLOURS[i % len(self._COLOURS)]
            (line,) = ax.plot([], [], color=colour, linewidth=1.2)

            self._axes[key]  = ax
            self._lines[key] = line

        # ── right panel ───────────────────────────────────────────────────
        for j, key in enumerate(self._actuation_keys):
            ax = fig.add_subplot(gs_right[j, 0])
            unit = _get_unit(data_dict, key)
            label_prefix = "Flap" if key.startswith("F") else "Engine"
            ylabel = f"{label_prefix} {key[1:]} [{unit}]" if unit else f"{label_prefix} {key[1:]}"
            ax.set_ylabel(ylabel, fontsize=7)
            ax.set_xlabel("Time [s]", fontsize=7)
            ax.set_xlim(0, self.total_sim_time)
            ax.tick_params(labelsize=7)
            ax.grid(True, linestyle="--", alpha=0.5)

            colour = self._COLOURS[(j + n_left) % len(self._COLOURS)]
            (line,) = ax.plot([], [], color=colour, linewidth=1.2)

            self._axes[key]  = ax
            self._lines[key] = line

        # ── column titles ─────────────────────────────────────────────────
        if n_left:
            fig.text(0.27, 0.91, "State Variables",
                     ha="center", fontsize=9, fontweight="bold")
        if n_right:
            fig.text(0.755, 0.91, "Actuation Channels",
                     ha="center", fontsize=9, fontweight="bold")
            


        # ── top-left info block (updated on every update() call) ──────────
        self._info_text = fig.text(
            0.01, 0.99, "",
            ha="left", va="top",
            fontsize=8, fontfamily="monospace",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow",
                      edgecolor="gray", alpha=0.8)
        )

        # ── convergence warning banner (hidden until needed) ───────────────
        self._warn_text = fig.text(
            0.5, 0.97, "",
            ha="center", va="center",
            fontsize=13, fontweight="bold", color="darkred",
            alpha=0.0,                          # invisible until triggered
            bbox=dict(boxstyle="round,pad=0.4", facecolor="mistyrose",
                      edgecolor="red", linewidth=2),
            zorder=10,
        )

        plt.ion()          # interactive mode → non-blocking show
        plt.show()
        self._fig = fig


# ── convenience wrapper ───────────────────────────────────────────────────────

def plot_aswing_dict(
    data_dict: dict,
    parameter_list: list,
    total_sim_time: float,
    export_path: str = None,
) -> "ASWINGLivePlotter":
    """
    One-liner post-processing helper.  Creates a plotter, calls update() once,
    optionally exports to PDF, and returns the plotter object.

    Example
    -------
    plotter = plot_aswing_dict(
        data_dict       = my_dict,
        parameter_list  = ["earth X", "earth Y", "Heading", "Bank"],
        total_sim_time  = 100.0,
        export_path     = "results.pdf",
    )
    """
    plotter = ASWINGLivePlotter(parameter_list, total_sim_time)
    plotter.update(data_dict, force_draw=True)
    if export_path:
        plotter.export(export_path)
    return plotter


# ── minimal smoke-test (no real files needed) ─────────────────────────────────
if __name__ == "__main__":

    # ── build a synthetic dictionary that mimics read_aswing_file output ──
    N = 80   # number of "timesteps" to fake
    t = np.linspace(0, 40, N)

    # Simulate divergence between t=18 s and t=26 s
    is_converged = [not (18 <= ti <= 26) for ti in t]

    def _fake_var(vals, unit=None):
        return {"values": list(vals), "unit": unit, "latex": None}

    fake_dict = {
        "ModelName": "Demo HALE Aircraft",
        "ModelStates": [],
        "ModelVariables": {
            "Time":        _fake_var(t,                                    "s"),
            "IsConverged": {"values": is_converged, "unit": None, "latex": None},
            "earth X":     _fake_var(t * 12.0,                            "m"),
            "earth Y":     _fake_var(np.sin(t * 0.3) * 5,                 "m"),
            "earth Z":     _fake_var(-t * 0.5 + np.random.randn(N)*0.2,   "m"),
            "Heading":     _fake_var(np.degrees(np.arctan(t*0.05)),        "deg"),
            "Elev.":       _fake_var(np.sin(t * 0.2) * 3,                 "deg"),
            "Bank":        _fake_var(np.cos(t * 0.15) * 8,                "deg"),
            "F1":          _fake_var(np.sin(t * 0.1) * 5,                 "deg"),
            "F2":          _fake_var(np.cos(t * 0.12) * 3,                "deg"),
            "E1":          _fake_var(9.5 + np.sin(t * 0.05),              ""),
            "E2":          _fake_var(10.5 + np.cos(t * 0.05),             ""),
            "E15":          {"values": [], "unit": None,"latex": None}
        }
    }


    param_list = ["earth X", "earth Y", "earth Z", "Heading", "Elev.", "Bank"]

    # ── iterative (live) demo ─────────────────────────────────────────────
    plotter = ASWINGLivePlotter(
        parameter_list=param_list,
        total_sim_time=40.0,
        refresh_interval=0.1,
    )

    print("Simulating live update (80 steps)…")
    for step in range(1, N + 1):
        # slice the fake dict as if only `step` timesteps have been written
        partial = {
            "ModelName": fake_dict["ModelName"],
            "ModelStates": [],
            "ModelVariables": {
                k: {"values": v["values"][:step], "unit": v["unit"], "latex": v["latex"]}
                for k, v in fake_dict["ModelVariables"].items()
            }
        }
        plotter.update(partial)
        time.sleep(0.03)   # simulate computation time

    plotter.export("aswing_live_demo.pdf")
    print("Done – press Enter to close the window.")
    input()
    plotter.close()