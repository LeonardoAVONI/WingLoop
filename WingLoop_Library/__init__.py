"""WingLoop 3.0: closed-loop control of ASWING time-transient simulations.

    from WingLoop_Library import WingLoop, Aswing_Director

Post-processing helpers (videos, strobe images, plots) are loaded on first use, so
importing the package does not import matplotlib, MATLAB or FMPy.
"""
from .WingLoop import WingLoop, __version__
from .Aswing_Director import Aswing_Director, ASWINGHangError, ASWINGCrashError
from .PyControl_IO import import_data_dict, export_data_dict

_HELPERS = {"VideoPlot_Generate_postscript_file", "Generate_Analysis_Videos", "Generate_Strobe_Plot",
            "Plot_Timeseries", "Create_Timeseries_GIF"}


def __getattr__(name):
    if name in _HELPERS:
        from . import ASW_Helpers
        return getattr(ASW_Helpers, name)
    raise AttributeError(f"module 'WingLoop_Library' has no attribute {name!r}")


__all__ = ["WingLoop", "Aswing_Director", "ASWINGHangError", "ASWINGCrashError",
           "import_data_dict", "export_data_dict", "__version__", *sorted(_HELPERS)]
