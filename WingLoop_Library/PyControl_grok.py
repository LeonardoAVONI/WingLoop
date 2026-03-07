"""
====================================================================================
Control_Library, Package Version

Author: Leonardo AVONI
Date: 08/11/2024
Email: avonileonardo@gmail.com

Last modified: 06/03/2026

====================================================================================

Description:
Step-by-step controller wrapper supporting:
  - python       : pure Python UserController class
  - matlab       : MATLAB UserController class via matlab.engine
  - simulink     : Simulink .slx model, stepped via eng.sim() with ExternalInput
  - simulink_fmu : FMU exported from Simulink, stepped via fmpy

HOW WORKSPACE VARIABLES WORK PER METHOD
────────────────────────────────────────
simulink (.slx):
  UserController.m is instantiated via the MATLAB engine.  Its three properties
  (workspace_scalar, workspace_string, workspace_matrix) are pushed into MATLAB's
  BASE workspace via assignin('base',...).  Simulink reads them from there at
  sim() time — exactly as if you had typed them in the MATLAB command window.
  Result: the .slx model always sees the current UserController values.

  NOTE ON IDENTICAL OUTPUTS: if your precomputed.mat stores the same numeric
  values as the default computation, Simulink outputs will be identical whether
  IsPrecomputed is True or False.  Check workspace_string in the log — if it
  shows 'test_matlab_workspace_database' the precomputed path is working.

simulink_fmu (.fmu):
  An FMU exported from Simulink bakes the Base Workspace variable VALUES into the
  binary at export time (they become FMI start values, not live parameters).
  After export there is NO live connection back to MATLAB's base workspace.

  Two strategies are available:
  ① rebuild_fmu=True  (recommended when workspace data changes between runs)
       PyControl pushes UserController values into the MATLAB base workspace,
       then calls Simulink's exportToFMU2CS to re-export the .fmu, then loads
       the freshly built FMU.  The rebuilt FMU now carries the correct start
       values.  Requires the matching .slx file in the same directory as the .fmu.
  ② rebuild_fmu=False (default — use the .fmu as-is)
       The FMU runs with whatever values were baked in at the last manual export.
       Use this when the FMU was already exported with the right data, or when
       re-exporting is too slow for your workflow.

====================================================================================
"""

import numpy as np
import os
import shutil
import sys
import warnings
from importlib import import_module

import matlab.engine
import fmpy
from fmpy import read_model_description, extract, instantiate_fmu


# ──────────────────────────────────────────────────────────────────────────────
# FMU helper
# ──────────────────────────────────────────────────────────────────────────────

class SimulinkFMUController:
    """
    Thin wrapper around an FMU exported from Simulink (Co-Simulation, FMI 2.0).
    Inputs  : statein[1] … statein[N]  (Real, per‑element)
    Outputs : F1, F2, F3, F4, E1, E2
    """

    def __init__(self, fmu_path: str, Dt: float = 0.01):
        self.fmu_path = fmu_path
        self.Dt       = Dt
        self.time     = 0.0

        self.model_description = read_model_description(self.fmu_path)
        self.unzipdir          = extract(self.fmu_path)
        self.fmu               = instantiate_fmu(
            unzipdir          = self.unzipdir,
            model_description = self.model_description,
            visible           = False,
            debug_logging     = False,
        )

        # Cache sorted input VRs once
        input_vars = [
            v for v in self.model_description.modelVariables
            if v.name.startswith('statein[')
        ]
        input_vars.sort(key=lambda v: v.valueReference)
        self._input_vrs    = [v.valueReference for v in input_vars]
        self._n_inputs     = len(input_vars)

        # Cache output VRs
        output_names = ['F1', 'F2', 'F3', 'F4', 'E1', 'E2']
        self._output_vrs = [
            next(
                (v.valueReference for v in self.model_description.modelVariables if v.name == n),
                None,
            )
            for n in output_names
        ]
        self._output_names = output_names

    # ------------------------------------------------------------------
    def set_parameters(self, params: dict):
        """
        Set FMU parameters (causality == 'parameter') from a dict {name: value}.
        Call this BEFORE the first step() call.
        """
        param_vrs = {
            v.name: v.valueReference
            for v in self.model_description.modelVariables
            if v.causality == 'parameter'
        }
        for key, value in params.items():
            if key in param_vrs:
                val = np.atleast_1d(np.array(value, dtype=np.float64)).flatten().tolist()
                self.fmu.setReal([param_vrs[key]], val)
            else:
                warnings.warn(f"[FMU] Parameter '{key}' not found in FMU — skipped.", UserWarning)

    # ------------------------------------------------------------------
    def step(self, instantaneous_state) -> dict:
        state_array = np.array(instantaneous_state, dtype=np.float64).flatten()

        if len(state_array) != self._n_inputs:
            raise ValueError(
                f"State length {len(state_array)} != FMU input count {self._n_inputs}"
            )

        self.fmu.setReal(self._input_vrs, state_array.tolist())
        self.fmu.doStep(
            currentCommunicationPoint = self.time,
            communicationStepSize     = self.Dt,
        )
        self.time += self.Dt

        values = self.fmu.getReal(self._output_vrs)
        return dict(zip(self._output_names, [float(v) for v in values]))

    # ------------------------------------------------------------------
    def terminate(self):
        if self.fmu:
            try:
                self.fmu.terminate()
            except Exception:
                pass
        if self.unzipdir and os.path.exists(self.unzipdir):
            shutil.rmtree(self.unzipdir, ignore_errors=True)


# ──────────────────────────────────────────────────────────────────────────────
# Main controller class
# ──────────────────────────────────────────────────────────────────────────────

class PyControl:
    """
    Unified step-by-step controller wrapper.

    Parameters
    ----------
    control_directory : str
        Folder that contains the controller file (and UserController.m for
        simulink / simulink_fmu).
    control_file : str
        Filename whose extension determines the method:
          .py  → python
          .m   → matlab
          .slx → simulink
          .fmu → simulink_fmu
    precomputed_file : str | None
        Optional filename (inside control_directory) for pre-computed data.
        If None or not found, UserController will compute its own defaults.
    library_path : str | None
        Root of WingLoop_Library (used for MATLAB addpath). Auto-detected if None.
    Dt : float
        Default timestep (seconds).
    """

    # ------------------------------------------------------------------
    def __init__(
        self,
        control_directory: str,
        control_file:      str,
        precomputed_file:  str | None = None,
        library_path:      str | None = None,
        Dt:                float       = 0.01,
        rebuild_fmu:       bool        = False,
    ):
        # ── paths ──────────────────────────────────────────────────────
        self.control_directory = os.path.abspath(os.path.normpath(control_directory))
        if not os.path.isdir(self.control_directory):
            raise NotADirectoryError(
                f"control_directory does not exist: {self.control_directory}"
            )

        self.control_file = control_file
        self.control_path = os.path.join(self.control_directory, control_file)
        if not os.path.isfile(self.control_path):
            raise FileNotFoundError(f"Control file not found: {self.control_path}")

        self.precomputed_path = None
        if precomputed_file:
            p = os.path.join(self.control_directory, precomputed_file)
            if os.path.isfile(p):
                self.precomputed_path = p
                print(f"[PyControl] Precomputed file found: {p}")
            else:
                warnings.warn(
                    f"Precomputed file not found → will compute defaults.\nMissing: {p}",
                    UserWarning,
                )

        # ── library path ───────────────────────────────────────────────
        if library_path is None:
            candidates = [
                os.path.expanduser("~/Desktop/01_GitHub/02_WingLoop/WingLoop_Library"),
                os.path.expanduser("~/Bureau/01 Github/02_WingLoop/WingLoop_Library"),
                os.path.expanduser("~/GitHub/02_WingLoop/WingLoop_Library"),
                os.getcwd(),
            ]
            library_path = next((p for p in candidates if os.path.isdir(p)), self.control_directory)
        self.library_path = os.path.abspath(library_path)

        # ── method ─────────────────────────────────────────────────────
        ext = os.path.splitext(control_file)[1].lower()
        method_map = {'.py': 'python', '.m': 'matlab', '.slx': 'simulink', '.fmu': 'simulink_fmu'}
        if ext not in method_map:
            raise ValueError(f"Unsupported extension '{ext}' in '{control_file}'")
        self.method = method_map[ext]
        self.control_model_name = os.path.splitext(control_file)[0]

        # ── state ──────────────────────────────────────────────────────
        self.Dt          = Dt
        self.time        = 0.0
        self.rebuild_fmu = rebuild_fmu

        self.eng                       = None
        self.fmu_controller            = None
        self.python_controller_instance = None
        self.matlab_controller_instance = None   # MATLAB UserController handle

        # ── MATLAB engine ──────────────────────────────────────────────
        # simulink_fmu only needs the engine when rebuild_fmu=True (to re-export).
        # Without rebuild, the FMU runs standalone — no MATLAB required.
        needs_engine = self.method in ('matlab', 'simulink') or \
                       (self.method == 'simulink_fmu' and self.rebuild_fmu)
        if needs_engine:
            print(f"[PyControl] Starting MATLAB engine for method='{self.method}' …")
            self.eng = matlab.engine.start_matlab("-nodesktop -nosplash")
            self.eng.cd(self.control_directory, nargout=0)
            self.eng.addpath(self.control_directory, nargout=0)
            self.eng.addpath(self.library_path, nargout=0)

        # ── Simulink model pre-load ────────────────────────────────────
        if self.method == 'simulink':
            try:
                self.eng.load_system(self.control_model_name, nargout=0)
                self.eng.set_param(self.control_model_name, 'ReturnWorkspaceOutputs', 'on', nargout=0)
                self.eng.set_param(self.control_model_name, 'SignalLogging',           'on', nargout=0)
            except Exception as exc:
                raise RuntimeError(
                    f"Failed to load Simulink model '{self.control_model_name}': {exc}"
                ) from exc

        # ── FMU instantiation ──────────────────────────────────────────
        elif self.method == 'simulink_fmu':
            self.fmu_controller = SimulinkFMUController(
                fmu_path = self.control_path,
                Dt       = self.Dt,
            )

        # ── Setup (load/compute UserController data) ───────────────────
        self._setup()

    # ------------------------------------------------------------------
    def _setup(self):
        """
        Instantiate UserController (Python or MATLAB) so it either loads
        pre-computed data or runs its default computation.
        For simulink / simulink_fmu the resulting workspace variables are
        pushed where they need to go (MATLAB base workspace / FMU params).
        """

        if self.method == 'python':
            sys.path.insert(0, self.control_directory)
            mod_name = self.control_model_name
            mod      = import_module(mod_name)
            try:
                UserControllerClass = getattr(mod, 'UserController')
            except AttributeError:
                raise AttributeError(
                    f"'{self.control_file}' must define a class named 'UserController'."
                )
            self.python_controller_instance = UserControllerClass(
                precomputed_file_path = self.precomputed_path or "",
            )

        elif self.method in ('matlab', 'simulink'):
            # Instantiate MATLAB UserController (handles precomputed or default)
            precomp_arg = self.precomputed_path if self.precomputed_path else ""
            print(f"[PyControl] Instantiating MATLAB UserController …")
            self.matlab_controller_instance = self.eng.UserController(precomp_arg)
            self._push_controller_to_base_workspace()

        elif self.method == 'simulink_fmu':
            if self.rebuild_fmu:
                # Need MATLAB to re-export: instantiate UserController, push to
                # base workspace so the .slx picks up the values on export.
                precomp_arg = self.precomputed_path if self.precomputed_path else ""
                print(f"[PyControl] Instantiating MATLAB UserController for FMU rebuild …")
                self.matlab_controller_instance = self.eng.UserController(precomp_arg)
                self._push_controller_to_base_workspace()
                self._push_controller_to_fmu()   # triggers re-export + reload
            else:
                # FMU runs standalone — skip MATLAB entirely, use FMU as-is.
                print(
                    "[PyControl] simulink_fmu: using FMU as-is (rebuild_fmu=False).\n"
                    "   Workspace variables are baked into the FMU from its last export.\n"
                    "   Set rebuild_fmu=True to re-export with current UserController values."
                )

    # ------------------------------------------------------------------
    def _extract_controller_data(self) -> dict:
        """
        Pull the three standard properties out of the MATLAB UserController handle
        and return them as plain Python / NumPy objects.

        Returns a dict with keys:
            'workspace_scalar'  : float
            'workspace_string'  : str
            'workspace_matrix'  : np.ndarray

        Uses a valid MATLAB variable name (no leading underscore).
        """
        # 'uc_handle' — no leading underscore, fully valid in MATLAB
        self.eng.workspace['uc_handle'] = self.matlab_controller_instance

        scalar = float(self.eng.eval("uc_handle.workspace_scalar", nargout=1))
        string = str(self.eng.eval("char(uc_handle.workspace_string)", nargout=1))
        matrix = np.array(self.eng.eval("uc_handle.workspace_matrix", nargout=1))

        return {
            'workspace_scalar': scalar,
            'workspace_string': string,
            'workspace_matrix': matrix,
        }

    # ------------------------------------------------------------------
    def _push_controller_to_base_workspace(self):
        """
        Push UserController's workspace properties into MATLAB's *base* workspace
        via assignin() so Simulink can read them by name during sim().

        UserController must expose:
            obj.workspace_scalar  (double)
            obj.workspace_string  (string / char)
            obj.workspace_matrix  (double matrix)
        """
        if self.matlab_controller_instance is None:
            return

        try:
            data = self._extract_controller_data()

            # assignin('base', ...) is the standard way to write to the base
            # workspace from within a MATLAB function / engine session.
            # We push uc_handle first so the eval calls have it available.
            self.eng.eval(
                "assignin('base', 'workspace_scalar', uc_handle.workspace_scalar);",
                nargout=0,
            )
            self.eng.eval(
                "assignin('base', 'workspace_string', uc_handle.workspace_string);",
                nargout=0,
            )
            self.eng.eval(
                "assignin('base', 'workspace_matrix', uc_handle.workspace_matrix);",
                nargout=0,
            )
            print(
                f"[PyControl] Pushed to MATLAB base workspace:\n"
                f"   workspace_scalar = {data['workspace_scalar']}\n"
                f"   workspace_string = {data['workspace_string']}\n"
                f"   workspace_matrix shape = {data['workspace_matrix'].shape}"
            )
            # Store for potential use by FMU
            self._controller_data = data

        except Exception as exc:
            raise RuntimeError(
                f"[PyControl] Failed to push UserController properties to base workspace.\n"
                f"Make sure UserController.m is on the MATLAB path and exposes "
                f"workspace_scalar, workspace_string, workspace_matrix.\nError: {exc}"
            ) from exc

    # ------------------------------------------------------------------
    def _push_controller_to_fmu(self):
        """
        Re-export the FMU from the matching .slx, with current base workspace
        values baked in, then reload self.fmu_controller from the fresh binary.

        Only called when rebuild_fmu=True. The .slx must have the same stem as
        the .fmu and live in the same directory.
        """
        slx_name = self.control_model_name
        slx_path = os.path.join(self.control_directory, slx_name + '.slx')
        if not os.path.isfile(slx_path):
            raise FileNotFoundError(
                f"[PyControl] rebuild_fmu=True but matching .slx not found: {slx_path}\n"
                f"   The .slx must live alongside the .fmu in: {self.control_directory}"
            )

        print(f"[PyControl] rebuild_fmu=True → re-exporting FMU from {slx_name}.slx …")

        # 1. Terminate and discard the old FMU instance
        self.fmu_controller.terminate()
        self.fmu_controller = None

        # 2. Load the .slx (no-op if already loaded)
        try:
            self.eng.load_system(slx_name, nargout=0)
        except Exception:
            pass

        # 3. Re-export to FMU (overwrites the existing .fmu).
        #    R2025b changed the API: 'CoSimulation' is now 'FMUType','CoSimulation'
        #    and FMI version must be specified explicitly as 'FMIVersion','2.0'.
        fmu_out_dir = self.control_directory.replace('\\', '/')
        try:
            self.eng.eval(
                f"exportToFMU2CS('{slx_name}', "
                f"'FMUType', 'CoSimulation', "
                f"'FMIVersion', '2.0', "
                f"'SaveDirectory', '{fmu_out_dir}');",
                nargout=0,
            )
        except Exception as exc:
            raise RuntimeError(
                f"[PyControl] FMU re-export failed: {exc}\n"
                f"   Make sure Simulink Coder and the FMI Kit are licensed and on the path."
            ) from exc

        # 4. Confirm output exists then reload
        new_fmu_path = os.path.join(self.control_directory, slx_name + '.fmu')
        if not os.path.isfile(new_fmu_path):
            raise FileNotFoundError(
                f"[PyControl] exportToFMU2CS ran but expected output not found: {new_fmu_path}"
            )

        self.fmu_controller = SimulinkFMUController(fmu_path=new_fmu_path, Dt=self.Dt)
        print(f"[PyControl] FMU rebuilt and reloaded from: {new_fmu_path}")

    # ------------------------------------------------------------------
    def PyControl_DoControllerStep(self, instantaneous_state, Dt: float | None = None) -> dict:
        """
        Run one control step.

        Parameters
        ----------
        instantaneous_state : array-like, shape (N,)
        Dt : float | None
            If None, uses self.Dt set at construction.

        Returns
        -------
        dict with keys F1, F2, F3, F4, E1, E2
        """
        if Dt is None:
            Dt = self.Dt

        # ── Python ────────────────────────────────────────────────────
        if self.method == 'python':
            if self.python_controller_instance is None:
                raise RuntimeError("Python controller not initialized.")
            output = self.python_controller_instance.step(instantaneous_state, Dt)

        # ── MATLAB ────────────────────────────────────────────────────
        elif self.method == 'matlab':
            state_ml = matlab.double(
                np.array(instantaneous_state, dtype=np.float64).flatten().tolist()
            )
            out_ml = self.eng.feval('step', self.matlab_controller_instance, state_ml, Dt, nargout=1)
            output = {k: float(out_ml[k]) for k in ('F1', 'F2', 'F3', 'F4', 'E1', 'E2')}

        # ── Simulink FMU ───────────────────────────────────────────────
        elif self.method == 'simulink_fmu':
            output = self.fmu_controller.step(instantaneous_state)

        # ── Simulink (.slx) ───────────────────────────────────────────
        elif self.method == 'simulink':
            output = self._simulink_step(instantaneous_state, Dt)

        else:
            raise ValueError(f"Unknown method: {self.method}")

        print(output)
        return output

    # ------------------------------------------------------------------
    def _simulink_step(self, instantaneous_state, Dt: float) -> dict:
        """
        Advance the Simulink model by one Dt using eng.sim() with ExternalInput.
        Mirrors the approach in previous_file.py that is known to work.
        """
        state_array = np.array(instantaneous_state, dtype=np.float64).flatten()

        t0 = self.time
        t1 = self.time + Dt

        # Build external-input matrix  [t, u_vector; t+Dt, u_vector]
        # Simulink's ExternalInput expects columns: [time, u1, u2, …]
        external_input = matlab.double(
            [[t0] + state_array.tolist(),
             [t1] + state_array.tolist()]
        )

        # Push to MATLAB workspace (NOT base workspace — eng.sim reads from
        # the engine workspace when given string variable names)
        self.eng.workspace['statein'] = external_input
        self.eng.workspace['Tstart']  = float(t0)
        self.eng.workspace['Tstop']   = float(t1)

        # Run the simulation for one step
        out = self.eng.sim(
            self.control_model_name,
            'StartTime',        'Tstart',
            'StopTime',         'Tstop',
            'LoadExternalInput','on',
            'ExternalInput',    'statein',
            nargout=1,
        )

        # Push SimulationOutput object so we can use eval() to extract fields
        self.eng.workspace['simOut'] = out

        # Extract outputs — try Dataset format first, fall back to Array format
        try:
            output = {
                "F1": float(self.eng.eval("simOut.F1.Data(end)", nargout=1)),
                "F2": float(self.eng.eval("simOut.F2.Data(end)", nargout=1)),
                "F3": float(self.eng.eval("simOut.F3.Data(end)", nargout=1)),
                "F4": float(self.eng.eval("simOut.F4.Data(end)", nargout=1)),
                "E1": float(self.eng.eval("simOut.E1.Data(end)", nargout=1)),
                "E2": float(self.eng.eval("simOut.E2.Data(end)", nargout=1)),
            }
        except Exception:
            # Fallback: Save format = Array
            output = {
                "F1": float(self.eng.eval("simOut.F1(end)", nargout=1)),
                "F2": float(self.eng.eval("simOut.F2(end)", nargout=1)),
                "F3": float(self.eng.eval("simOut.F3(end)", nargout=1)),
                "F4": float(self.eng.eval("simOut.F4(end)", nargout=1)),
                "E1": float(self.eng.eval("simOut.E1(end)", nargout=1)),
                "E2": float(self.eng.eval("simOut.E2(end)", nargout=1)),
            }

        self.time += Dt
        return output

    # ------------------------------------------------------------------
    def terminate(self):
        """Release all resources (FMU + MATLAB engine)."""
        if self.fmu_controller:
            self.fmu_controller.terminate()
        if self.eng:
            try:
                self.eng.quit()
            except Exception:
                pass


# ──────────────────────────────────────────────────────────────────────────────
# Quick test
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":

    """ 
    matlab (both)
    python (both)
    simulink (both)
    fmu works (both)

    Are libraries actually useful???

    """

    method        = "matlab"   # change to: python | matlab | simulink | simulink_fmu
    IsPrecomputed = False

    # rebuild_fmu only relevant for simulink_fmu.
    # Set True to re-export the FMU from the .slx after pushing UserController
    # workspace values — required if the FMU was exported before you changed
    # workspace_scalar / workspace_matrix, or to verify precomputed vs computed.
    # Set False (default) to use the .fmu binary as-is.
    RebuildFMU = True

    test_instantaneous_state = ((np.arange(1945) + 1) * 10).astype(float)
    test_Dt = 0.01

    precomputed_filename = None
    if IsPrecomputed:
        precomputed_filename = "precomputed_python.npz" if method == "python" else "precomputed.mat"

    # Anchor base_dir relative to THIS file's location, not the process cwd.
    # This makes the script work whether launched from WingLoop_Library/, a
    # subdirectory, or via VS Code's debugger (which may set cwd elsewhere).
    _this_dir = os.path.dirname(os.path.abspath(__file__))
    base_dir  = os.path.join(_this_dir, "test_files", "test_controllers")

    dir_map = {
        "python":        os.path.join(base_dir, "python"),
        "matlab":        os.path.join(base_dir, "matlab"),
        "simulink":      os.path.join(base_dir, "simulink"),
        "simulink_fmu":  os.path.join(base_dir, "simulink"),
    }
    file_map = {
        "python":        "python_test_controller.py",
        "matlab":        "UserController.m",
        "simulink":      "simulink_test_controller.slx",
        "simulink_fmu":  "simulink_test_controller.fmu",
    }

    ControlInstance = PyControl(
        control_directory = dir_map[method],
        control_file      = file_map[method],
        precomputed_file  = precomputed_filename,
        Dt                = test_Dt,
        rebuild_fmu       = RebuildFMU,
    )

    ControlInstance.PyControl_DoControllerStep(test_instantaneous_state, Dt=test_Dt)