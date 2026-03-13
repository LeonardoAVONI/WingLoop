# =============================================================================
# WingLoop — PyControl
# =============================================================================
# Copyright (c) 2024-2026 Leonardo Avoni (avonileonardo@gmail.com)
#
# This file is part of WingLoop.
# WingLoop is licensed under CC BY-NC-SA 4.0 (Non-Commercial use only).
# Full license: https://creativecommons.org/licenses/by-nc-sa/4.0/
#
# For commercial use, contact the author: avonileonardo@gmail.com
#
# If you use WingLoop in academic work, please cite:
#   Avoni et al., "Enhancing ASWING Flight Dynamics Simulations with
#   Closed-Loop Control for Flexible Aircraft," AIAA 2025-3425.
#   https://arc.aiaa.org/doi/10.2514/6.2025-3425
# =============================================================================

"""
====================================================================================
WingLoop Library — PyControl
Unified Controller Interface for WingLoop
====================================================================================

Author: Leonardo Avoni
Email: avonileonardo@gmail.com
Initial release: 08 Nov 2024
Last modified: 08 Mar 2026

------------------------------------------------------------------------------------
Overview
------------------------------------------------------------------------------------
PyControl provides a unified interface for executing control algorithms within the
WingLoop aeroservoelastic simulation framework.

The module acts as a controller wrapper that allows different controller
implementations to be executed during a WingLoop simulation step. The
backend is automatically selected based on the extension of the supplied
control file.

Supported controller backends:

    .py   → Python controller
    .m    → MATLAB controller (via matlab.engine)
    .slx  → Simulink model stepped through the MATLAB engine
    .fmu  → FMU (FMI 2.0 Co-Simulation) exported from Simulink and executed via FMPy

All controller types share the same runtime interface, allowing WingLoop to
interact with them in a consistent way.

------------------------------------------------------------------------------------
Typical Usage
------------------------------------------------------------------------------------

    ctrl = PyControl(control_directory, control_file, ...)
    output = ctrl.PyControl_DoControllerStep(state, Dt=0.01)
    ctrl.terminate()

Where:
    state   : current aircraft state vector
    Dt      : simulation timestep
    output  : dictionary of controller outputs (e.g. {'F1': ..., 'E1': ...})

------------------------------------------------------------------------------------
Controller Output Convention
------------------------------------------------------------------------------------
Controller outputs are returned as a Python dictionary whose keys correspond
to actuator or control signals.

Typical signals include:

    F1 - F20 : control surface / actuator commands
    E1 - E20 : engine or auxiliary control outputs

The actual outputs are discovered dynamically depending on the controller
implementation.

------------------------------------------------------------------------------------
Supported Controller Types
------------------------------------------------------------------------------------

Python Controllers (.py)
    Must define a class named `UserController` containing:

        __init__(self, precomputed_file_path)
        step(self, instantaneous_state, Dt) → dict

    The step() method must return a dictionary of output signals.

MATLAB Controllers (.m)
    Must define a MATLAB class named `UserController` with an equivalent
    step() method. The controller is executed through the MATLAB engine.

Simulink Models (.slx)
    Simulink models are stepped using `eng.sim()` with an ExternalInput vector
    representing the current plant state.

    The model must save output signals to the SimulationOutput object
    (ReturnWorkspaceOutputs = on).

FMU Controllers (.fmu)
    Simulink models exported as FMI 2.0 Co-Simulation FMUs are executed through
    the FMPy library. Input ports correspond to the state vector, while output
    variables are discovered automatically from the FMU model description.

------------------------------------------------------------------------------------
Workspace Variable Handling
------------------------------------------------------------------------------------

Python
    Precomputed controller data is passed to the UserController constructor
    through a file path. If no file is provided, the controller initializes its
    own default parameters.

MATLAB / Simulink (.slx)
    UserController.m is fully responsible for populating the MATLAB base workspace.
    Any variable that Simulink needs at sim() time must be pushed inside the
    UserController constructor using:

        assignin('base', 'my_variable', value);

    PyControl does not inspect UserController properties or mirror them to the
    base workspace.  This decouples the controller's internal design from the
    infrastructure layer.

Simulink FMU (.fmu)
    When a Simulink model is exported as an FMU, workspace variables are baked
    into the FMU as initial parameter values.

Two execution modes are available:

    rebuild_fmu = True
        The FMU is re-exported from the corresponding `.slx` file before
        execution. UserController's constructor runs first (performing its
        assignin calls), then exportToFMU2CS captures current base workspace
        values as FMI start values in the binary.

    rebuild_fmu = False
        The FMU runs with the parameters embedded during the previous export.
        This results in faster startup but requires manual FMU regeneration if
        parameters change.

------------------------------------------------------------------------------------
Main Components
------------------------------------------------------------------------------------

PyControl
    Unified controller interface used by WingLoop to execute control laws.

SimulinkFMUController
    Lightweight wrapper that manages the lifecycle of an FMU exported from
    Simulink, including instantiation, stepping, and cleanup.

------------------------------------------------------------------------------------
Testing
------------------------------------------------------------------------------------
Example test controllers are available in:

    test_files/test_controllers/

These examples demonstrate controller implementations for all supported
backends.

------------------------------------------------------------------------------------
Known Limitations / Future Improvements
------------------------------------------------------------------------------------

• Time synchronization differences may occur when using the FMU backend
  compared to Python, MATLAB, or Simulink execution modes.

• Additional diagnostics for FMU parameter synchronization could be added.

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

    Manages the full FMU lifecycle: parse the model description, extract the
    binary to a temp directory, instantiate the co-simulation object, run
    communication steps, and clean up on termination.

    Inputs  : statein[1] … statein[N]  — flat state vector, one port per element.
    Outputs : all variables with causality='output', discovered automatically
              from the model description (no hardcoded signal names).

    Parameters
    ----------
    fmu_path : str
        Absolute path to the .fmu file.
    Dt : float
        Communication step size in seconds (default 0.01).
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
        
        print("[PyControl] Setting the timestep in FMU to ",self.Dt)

        # Cache sorted input VRs once
        input_vars = [
            v for v in self.model_description.modelVariables
            if v.name.startswith('statein[')
        ]
        input_vars.sort(key=lambda v: v.valueReference)
        self._input_vrs    = [v.valueReference for v in input_vars]
        self._n_inputs     = len(input_vars)

        # Auto-discover all output variables from the model description.
        # Any variable with causality='output' is included — no hardcoded names.
        output_vars = sorted(
            [v for v in self.model_description.modelVariables if v.causality == 'output'],
            key=lambda v: v.valueReference,
        )
        self._output_vrs   = [v.valueReference for v in output_vars]
        self._output_names = [v.name for v in output_vars]
        print(f"[FMU] Auto-discovered {len(self._output_names)} output(s): {self._output_names}")

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
        """
        Advance the FMU by one communication step and return all output values.

        Sets the statein input ports, calls doStep from self.time to
        self.time + Dt, increments self.time, then reads all output ports.

        Parameters
        ----------
        instantaneous_state : array-like, shape (N,)
            Current state vector.  Must have exactly self._n_inputs elements.

        Returns
        -------
        dict
            {signal_name: float} for every auto-discovered output variable.

        Raises
        ------
        ValueError
            If len(instantaneous_state) != self._n_inputs.
        """
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
        """
        Terminate the FMU instance and delete the temporary extraction directory.

        Safe to call multiple times; subsequent calls are no-ops.
        """
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
        Folder containing the controller file.  Added to the MATLAB path
        automatically for MATLAB-based backends.
    control_file : str
        Filename whose extension selects the backend:
          .py  → python
          .m   → matlab
          .slx → simulink
          .fmu → simulink_fmu
    precomputed_file : str | None
        Optional filename (inside control_directory) for pre-computed
        initialisation data.  Passed to UserController; if None or not found
        the controller computes its own defaults.
    Dt : float
        Default communication / simulation timestep in seconds (default 0.01).
    rebuild_fmu : bool
        simulink_fmu only.  Re-export the FMU from the matching .slx before
        running so that current UserController workspace values are baked in.
        Requires the .slx to be present alongside the .fmu.  Default False.
    show_simulink : bool
        simulink only.  If True, opens the Simulink block-diagram window so
        that Scopes and sinks display live data.  Requires a full MATLAB desktop
        session (JVM + display).  Default False.
    """

    # ------------------------------------------------------------------
    def __init__(
        self,
        control_directory: str,
        control_file:      str,
        precomputed_file:  str | None = None,
        Dt:                float       = 0.01,
        rebuild_fmu:       bool        = False,
        show_simulink:     bool        = False,
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

        # ── method ─────────────────────────────────────────────────────
        ext = os.path.splitext(control_file)[1].lower()
        method_map = {'.py': 'python', '.m': 'matlab', '.slx': 'simulink', '.fmu': 'simulink_fmu'}
        if ext not in method_map:
            raise ValueError(f"Unsupported extension '{ext}' in '{control_file}'")
        self.method = method_map[ext]
        self.control_model_name = os.path.splitext(control_file)[0]

        # ── state ──────────────────────────────────────────────────────
        self.Dt            = Dt
        self.time          = 0.0
        self.rebuild_fmu   = rebuild_fmu
        self.show_simulink = show_simulink

        self.eng                        = None
        self.fmu_controller             = None
        self.python_controller_instance = None
        self.matlab_controller_instance = None

        # ── MATLAB engine ──────────────────────────────────────────────
        # simulink_fmu only needs the engine when rebuild_fmu=True (to re-export).
        # Without rebuild, the FMU runs standalone — no MATLAB required.
        needs_engine = self.method in ('matlab', 'simulink') or \
                       (self.method == 'simulink_fmu' and self.rebuild_fmu)
        if needs_engine:
            print(f"[PyControl] Starting MATLAB engine for method='{self.method}' …")
            if self.method == 'matlab':
                # Pure MATLAB: no JVM needed (faster startup, ~3-5 s saved)
                flags = "-nodesktop -nosplash -nojvm"
            elif self.show_simulink:
                # Simulink with visible GUI: needs JVM + desktop for scopes/windows
                flags = "-nosplash"
            else:
                # Simulink headless: suppress desktop, keep JVM (Simulink requires it)
                flags = "-nodesktop -nosplash"
            self.eng = matlab.engine.start_matlab(flags)
            self.eng.cd(self.control_directory, nargout=0)
            self.eng.addpath(self.control_directory, nargout=0)

        # ── Simulink model pre-load ────────────────────────────────────
        if self.method == 'simulink':
            try:
                self.eng.load_system(self.control_model_name, nargout=0)
                
                actual_solver = self.eng.get_param(self.control_model_name, 'Solver', nargout=1)
                if actual_solver not in ('FixedStepDiscrete',):
                    import warnings
                    warnings.warn(
                        f"[PyControl] SLX solver is '{actual_solver}'. "
                        f"FMU export requires a fixed-step solver. "
                        f"Results may differ if model has continuous states.",
                        UserWarning
                    )
                self.eng.set_param(self.control_model_name, 'ReturnWorkspaceOutputs', 'on',   nargout=0)
                self.eng.set_param(self.control_model_name, 'SignalLogging',           'on',   nargout=0)
                self.eng.set_param(self.control_model_name, 'StartTime',               '0.0', nargout=0)
                # ── NEW: sync Simulink solver timestep with Python Dt ──────────
                # ──────────────────────────────────────────────────────────────
                #self.eng.set_param(self.control_model_name, 'FastRestart', 'on', nargout=0)
                
                self.eng.set_param(self.control_model_name, 'Solver',    'FixedStepDiscrete', nargout=0)
                self.eng.set_param(self.control_model_name, 'FixedStep', str(self.Dt),        nargout=0)
                # FastRestart blocks OutputTimes and forces only 1 logged sample → DISABLE it
                # self.eng.set_param(self.control_model_name, 'FastRestart', 'on', nargout=0)  # ← COMMENT THIS OUT               

                # Enforce self.Dt on every discrete block — overrides whatever is hardcoded in the .slx
                all_blocks = self.eng.find_system(self.control_model_name, 'Type', 'Block', nargout=1)
                for blk in all_blocks:
                    try:
                        st = self.eng.get_param(blk, 'SampleTime', nargout=1)
                        if float(st) > 0:          # skip inherited(-1), triggered(-2), continuous(0)
                            self.eng.set_param(blk, 'SampleTime', str(self.Dt), nargout=0)
                    except Exception:
                        pass
                
                
                if self.show_simulink:
                    # open_system() brings up the block diagram window.
                    # Scopes and other sinks will display live data as sim() runs.
                    self.eng.open_system(self.control_model_name, nargout=0)
                    print(f"[PyControl] Simulink model '{self.control_model_name}' opened graphically.")
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
        
        if self.method == 'simulink':
            self.eng.eval(f"assignin('base', 'Nfilter', {1.0 / self.Dt});", nargout=0)
            print(f"[PyControl] Nfilter = {1.0 / self.Dt:.4f} pushed to MATLAB base workspace (N·Dt = 1.0)")

    # ------------------------------------------------------------------
    def _setup(self):
        """
        Initialise the UserController and distribute workspace data.

        Behaviour per backend:

        python
            Imports the .py module, instantiates UserController, and passes
            the precomputed file path (empty string if not available).
        matlab / simulink
            Instantiates UserController.m via the MATLAB engine.  UserController
            is responsible for pushing any variables it needs into the MATLAB base
            workspace by calling assignin('base', ...) directly inside its
            constructor.  PyControl does not inspect or mirror any UserController
            properties — the contract is entirely owned by UserController.m.
        simulink_fmu, rebuild_fmu=True
            Instantiates UserController.m (whose constructor runs assignin calls),
            then calls _push_controller_to_fmu() which re-exports the .slx to a
            fresh .fmu — the export captures current base workspace values as FMI
            start values — and reloads fmu_controller.
        simulink_fmu, rebuild_fmu=False
            Skips MATLAB entirely; the FMU runs with values baked in at the
            last manual export.
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
            # Instantiate MATLAB UserController (handles precomputed or default).
            # UserController.__init__ is responsible for pushing any variables it needs
            # into the MATLAB base workspace via assignin('base', ...) directly.
            precomp_arg = self.precomputed_path if self.precomputed_path else ""
            print(f"[PyControl] Instantiating MATLAB UserController …")
            self.matlab_controller_instance = self.eng.UserController(precomp_arg)

        elif self.method == 'simulink_fmu':
            if self.rebuild_fmu:
                # Need MATLAB to re-export: instantiate UserController so it pushes
                # all required variables to the MATLAB base workspace via its own
                # assignin('base', ...) calls, then re-export the FMU.
                precomp_arg = self.precomputed_path if self.precomputed_path else ""
                print(f"[PyControl] Instantiating MATLAB UserController for FMU rebuild …")
                self.matlab_controller_instance = self.eng.UserController(precomp_arg)
                # Push Nfilter before export so it gets baked into the FMU binary
                self.eng.eval(f"assignin('base', 'Nfilter', {1.0 / self.Dt});", nargout=0)
                print(f"[PyControl] Nfilter = {1.0 / self.Dt:.4f} pushed to base workspace for FMU export (N·Dt = 1.0)")
                self._push_controller_to_fmu()   # triggers re-export + reload
            else:
                # FMU runs standalone — skip MATLAB entirely, use FMU as-is.
                print(
                    "[PyControl] simulink_fmu: using FMU as-is (rebuild_fmu=False).\n"
                    "   Workspace variables are baked into the FMU from its last export.\n"
                    "   Set rebuild_fmu=True to re-export with current UserController values."
                )

    # ------------------------------------------------------------------
    def _push_controller_to_fmu(self):
        """
        Re-export the FMU from the matching .slx and reload self.fmu_controller.

        Called only when rebuild_fmu=True.  Assumes UserController.__init__ has
        already run and pushed the required variables to the MATLAB base workspace
        via assignin('base', ...) calls, so that exportToFMU2CS can capture them
        as FMI start values.

        Procedure:
            1. Terminate and discard the existing FMU instance.
            2. Load the .slx (no-op if already loaded).
            3. Call exportToFMU2CS to overwrite the .fmu with one that has
               current base workspace values baked in as FMI start values.
            4. Verify the output file exists, then construct a new
               SimulinkFMUController from it.

        Raises
        ------
        FileNotFoundError
            If the .slx is not found alongside the .fmu, or if exportToFMU2CS
            ran but produced no output file.
        RuntimeError
            If exportToFMU2CS fails (e.g. missing Simulink Coder licence).
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
        
        # 2.5 Timestep modification
        # Enforce self.Dt in the .slx before compiling the FMU binary
        self.eng.set_param(slx_name, 'Solver',    'FixedStepDiscrete', nargout=0)
        self.eng.set_param(slx_name, 'FixedStep', str(self.Dt),        nargout=0)
        all_blocks = self.eng.find_system(slx_name, 'Type', 'Block', nargout=1)
        for blk in all_blocks:
            try:
                st = self.eng.get_param(blk, 'SampleTime', nargout=1)
                if float(st) > 0:
                    self.eng.set_param(blk, 'SampleTime', str(self.Dt), nargout=0)
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
        Execute one control step and return the controller outputs.

        Dispatches to the appropriate backend and returns a dict whose keys are
        the output signal names discovered from that backend.  Output names are
        never hardcoded; they are found once on the first call and cached.

        Parameters
        ----------
        instantaneous_state : array-like, shape (N,)
            Current plant state vector fed to the controller.
        Dt : float | None
            Timestep for this step in seconds.  If None, uses self.Dt.

        Returns
        -------
        dict
            {signal_name: float} for every output exposed by the controller.
            Keys follow the pattern F1-F20 (force/actuator commands) and
            E1-E20 (engine or auxiliary outputs), depending on what the
            controller defines.
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
            state_np = np.asarray(instantaneous_state, dtype=np.float64).ravel()
            state_ml = matlab.double(state_np)  # no .tolist() needed
            #state_ml = matlab.double(
            #    np.array(instantaneous_state, dtype=np.float64).flatten().tolist()
            #)
            out_ml = self.eng.feval('step', self.matlab_controller_instance, state_ml, Dt, nargout=1)
            # Iterate all keys returned by the MATLAB step() function rather than
            # a hardcoded list — any F/E output added to UserController.m is included.
            output = {k: float(out_ml[k]) for k in out_ml.keys()}

        # ── Simulink FMU ───────────────────────────────────────────────
        elif self.method == 'simulink_fmu':
            output = self.fmu_controller.step(instantaneous_state)

        # ── Simulink (.slx) ───────────────────────────────────────────
        elif self.method == 'simulink':
            output = self._simulink_step(instantaneous_state, Dt)

        else:
            raise ValueError(f"Unknown method: {self.method}")

        return output





    def _simulink_step(self, instantaneous_state, Dt: float) -> dict:
        state_array = np.array(instantaneous_state, dtype=np.float64).flatten()
        t0 = self.time
        t1 = self.time + Dt

        self.eng.workspace['wl_step_data'] = self.eng.eval(
            f"[{t0}, {', '.join(map(str, state_array))}; "
            f"{t1}, {', '.join(map(str, state_array))}]",
            nargout=1
        )
        self.eng.eval("statein = wl_step_data;", nargout=0)

        sim_args = [
            self.control_model_name,
            'StartTime',         str(t0),
            'StopTime',          str(t1),
            'LoadExternalInput', 'on',
            'ExternalInput',     'statein',
            'SaveFormat',        'Dataset',
            'SaveFinalState',    'on',
            'FinalStateName',    'wl_op_state',
        ]
        if hasattr(self, '_simulink_state'):
            self.eng.workspace['wl_initial_state'] = self._simulink_state
            sim_args.extend(['LoadInitialState', 'on', 'InitialState', 'wl_initial_state'])

        out = self.eng.sim(*sim_args, nargout=1)
        self.eng.workspace['simOut'] = out
        self._simulink_state = self.eng.eval("simOut.wl_op_state", nargout=1)

        if not hasattr(self, '_simulink_signal_names') or not self._simulink_signal_names:
            self._simulink_signal_names = self._probe_simulink_signals()
            print(f"[PyControl] Simulink signals found: {self._simulink_signal_names}")

        output = {}
        for name in self._simulink_signal_names:
            val = self.eng.eval(f"simOut.{name}.Data(1)", nargout=1)
            output[name] = float(np.array(val).flatten()[0])

        self.time += Dt
        return output
        
    
    
        
    # ------------------------------------------------------------------
    def _probe_simulink_signals(self) -> list:
        """
        Discover which F/E output signals exist in the current simOut object.

        Probes F1–F20 and E1–E20 using MATLAB's isfield() and isprop() so that
        no error messages are printed to stdout for absent signals.  Called once
        after the first sim(); the result is stored in self._simulink_signal_names
        and reused on every subsequent step.

        Returns
        -------
        list[str]
            Ordered list of signal names present in simOut,
            e.g. ['F1', 'F2', 'F3', 'F4', 'E1', 'E2', 'E15'].
        """
        candidates = [f"F{i}" for i in range(1, 21)] + [f"E{i}" for i in range(1, 21)]
        found = []
        for n in candidates:
            # Check existence silently before accessing — avoids MATLAB printing
            # "Unrecognized field name" errors to stdout for missing signals.
            exists = bool(self.eng.eval(
                f"isfield(simOut, '{n}') || "
                f"(isobject(simOut) && isprop(simOut, '{n}'))",
                nargout=1,
            ))
            if exists:
                found.append(n)
        return found

    # ------------------------------------------------------------------
    def terminate(self):
        """
        Release all resources held by this controller instance.

        FMU backend  : calls SimulinkFMUController.terminate(), which terminates
                       the co-simulation and deletes the temp extraction directory.
        MATLAB backend: quits the MATLAB engine process.  If show_simulink=True
                       the engine is intentionally left running so the Simulink
                       window stays open; the user must close MATLAB manually.

        Safe to call even if initialisation did not complete fully.
        """
        if self.fmu_controller:
            self.fmu_controller.terminate()
        if self.eng:
            if self.show_simulink:
                # Leave MATLAB/Simulink window open for inspection.
                # The engine process will stay alive until the user closes MATLAB.
                print("[PyControl] Simulink window left open (show_simulink=True). Close MATLAB manually when done.")
            else:
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

    prints are actually ok
    RebuildFMU oly if FMU option is used (yes)
    graphics working? (yes)
    """

    method        = "simulink"   # change to: python | matlab | simulink | simulink_fmu
    IsPrecomputed = True
    RebuildFMU    = True   # simulink_fmu only
    ShowSimulink  = True   # simulink only: open block-diagram + scopes graphically

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
        show_simulink     = ShowSimulink,
    )

    for i in range(50):
        output = ControlInstance.PyControl_DoControllerStep(test_instantaneous_state, Dt=test_Dt)
        print(output)