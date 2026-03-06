# python_test_controller.py

import numpy as np
import sys
import os
from importlib import import_module

class UserController:
    """
    User-defined controller.
    All initialization logic (precomputed or startup computation) lives here.
    """

    def __init__(self, precomputed_file_path: str | None = None):
        self.workspace_scalar_py = None
        self.workspace_string_py = None
        self.workspace_matrix_py = None

        # Priority: precomputed > compute from startup
        if precomputed_file_path and os.path.isfile(precomputed_file_path):
            self._load_precomputed(precomputed_file_path)
        else:
            self._compute_from_startup()

        print("[UserController] Initialized with following values:")
        print("     self.workspace_scalar_py:",self.workspace_scalar_py)
        print("     self.workspace_string_py:",self.workspace_string_py)
        print("     self.workspace_matrix_py:")
        print(self.workspace_matrix_py)
        print("\n")

    def _load_precomputed(self, path: str):
        print(f"\n[UserController] Loading precomputed from: {path}")
        data = np.load(path, allow_pickle=True)        
        self.workspace_scalar_py = data.get('workspace_scalar_py', 0.0)
        self.workspace_string_py = data.get('workspace_string_py', "")
        self.workspace_matrix_py = data.get('workspace_matrix_py', np.zeros((1,1)))


    def _compute_from_startup(self):

        print(f"\n[UserController] Computing initial data...")
        data = {}
        data["workspace_scalar_py"] = 1234.5678
        data['workspace_string_py'] = "test_matlab_workspace"
        data['workspace_matrix_py'] = np.eye(5)
        
        self.workspace_scalar_py = data["workspace_scalar_py"]
        self.workspace_string_py = data['workspace_string_py']
        self.workspace_matrix_py = data['workspace_matrix_py']


    def step(self, instantaneous_state, Dt):
        """
        Main controller step – user implements the logic here.
        Can use self.workspace_xxx attributes freely.
        """
        output = {}
        output["F1"] = instantaneous_state[0]
        output["F2"] = instantaneous_state[1]
        output["F3"] = instantaneous_state[2]
        output["F4"] = instantaneous_state[3]
        output["E1"] = instantaneous_state[4]
        output["E2"] = instantaneous_state[5]

        return output