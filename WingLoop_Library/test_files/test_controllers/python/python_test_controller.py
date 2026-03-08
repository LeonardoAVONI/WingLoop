"""
====================================================================================
WingLoop Library — UserController (Python)

Author: Leonardo AVONI
Date: 08/11/2024
Email: avonileonardo@gmail.com

Last modified: 08/03/2026

====================================================================================

Description:
    Pure Python implementation of the UserController interface expected by
    PyControl for the 'python' backend.

    On construction the class either loads pre-computed workspace data from a
    .npz file, or runs _compute_from_startup() to produce default values.  The
    step() method is called once per simulation timestep by PyControl and must
    return a dict whose keys are the controller output signals (F1–F20, E1–E20
    as required by the application).

====================================================================================
"""

import numpy as np
import os


class UserController:
    """
    User-defined controller for the PyControl 'python' backend.

    Initialisation loads workspace data either from a pre-computed .npz file
    or by running the default computation.  The step() method implements the
    per-timestep control logic and can freely access the workspace attributes.

    Attributes
    ----------
    workspace_scalar_py : float
        Scalar workspace value, loaded or computed at construction.
    workspace_string_py : str
        String workspace value, loaded or computed at construction.
    workspace_matrix_py : np.ndarray
        Matrix workspace value, loaded or computed at construction.
    simulationtime : float
        Accumulated simulation time in seconds, incremented in step().
    """

    # ------------------------------------------------------------------
    def __init__(self, precomputed_file_path: str | None = None):
        """
        Initialise workspace data and reset the simulation clock.

        If precomputed_file_path points to an existing .npz file the workspace
        attributes are loaded from it via _load_precomputed().  Otherwise
        _compute_from_startup() is called to produce default values.
        simulationtime is always reset to 0.0 regardless of the source.

        Parameters
        ----------
        precomputed_file_path : str | None
            Path to a .npz file containing workspace_scalar_py,
            workspace_string_py, and workspace_matrix_py arrays.
            Pass None or an empty string to use computed defaults.
        """
        self.workspace_scalar_py = None
        self.workspace_string_py = None
        self.workspace_matrix_py = None

        if precomputed_file_path and os.path.isfile(precomputed_file_path):
            self._load_precomputed(precomputed_file_path)
        else:
            self._compute_from_startup()

        self.simulationtime = 0.0

        print("[UserController] Initialized with following values:")
        print("     self.workspace_scalar_py:", self.workspace_scalar_py)
        print("     self.workspace_string_py:", self.workspace_string_py)
        print("     self.workspace_matrix_py:")
        print(self.workspace_matrix_py)
        print("\n")

    # ------------------------------------------------------------------
    def _load_precomputed(self, path: str):
        """
        Load workspace attributes from a .npz file.

        Reads workspace_scalar_py, workspace_string_py, and
        workspace_matrix_py from the archive.  Any key absent from the file
        falls back to a safe default (0.0, "", or a (1,1) zero matrix).

        Parameters
        ----------
        path : str
            Path to the .npz archive to load.
        """
        print(f"\n[UserController] Loading precomputed from: {path}")
        data = np.load(path, allow_pickle=True)
        self.workspace_scalar_py = data.get('workspace_scalar_py', 0.0)
        self.workspace_string_py = data.get('workspace_string_py', "")
        self.workspace_matrix_py = data.get('workspace_matrix_py', np.zeros((1, 1)))

    # ------------------------------------------------------------------
    def _compute_from_startup(self):
        """
        Compute default workspace attributes.

        Called when no valid precomputed file is supplied.  Sets
        workspace_scalar_py, workspace_string_py, and workspace_matrix_py
        to hard-coded test values.  Replace with real computation as needed.
        """
        print(f"\n[UserController] Computing initial data...")
        self.workspace_scalar_py = 1234.5678
        self.workspace_string_py = "test_matlab_workspace"
        self.workspace_matrix_py = self.workspace_scalar_py * np.eye(5)

    # ------------------------------------------------------------------
    def step(self, instantaneous_state, Dt):
        """
        Execute one controller timestep.

        Called once per simulation step by PyControl.  Increments
        simulationtime, applies control logic using the workspace attributes
        and the current state, and returns a dict of output signals.  Add or
        remove output keys (F1–F20, E1–E20) to match the application;
        PyControl discovers them automatically.

        Parameters
        ----------
        instantaneous_state : array-like, shape (N,)
            Current plant state vector.
        Dt : float
            Timestep duration in seconds.

        Returns
        -------
        dict
            {signal_name: value} for every controller output signal.
        """
        self.simulationtime += Dt

        output = {}
        output["F1"]  = instantaneous_state[0] + self.workspace_scalar_py
        output["F2"]  = instantaneous_state[1] + self.workspace_scalar_py
        output["F3"]  = instantaneous_state[2]
        output["F4"]  = instantaneous_state[3]
        output["E1"]  = instantaneous_state[4]
        output["E2"]  = instantaneous_state[5]
        output["E15"] = self.simulationtime
        return output