"""
====================================================================================
Control_Library, Package Version

Author: Leonardo AVONI
Date: 08/11/2024
Email: avonileonardo@gmail.com

Last modified: 29/05/2025

====================================================================================

"""

import numpy as np
import matplotlib.pyplot as plt
import os
#from WingLoop_Library.PyControl_Text2Python import extract_states_vector
from scipy.integrate import trapezoid
import matlab.engine
import fmpy
import shutil
from fmpy import read_model_description, extract, instantiate_fmu
import sys
from importlib import import_module
import scipy.io

import warnings

class PyControl:
    """
    Control class, containing the various control laws and history tracking of the variables used in Python
    """
    def __init__(
        self,
        control_directory: str,
        startup_file: str,
        control_file: str,
        precomputed_file: str | None = None,
        library_path: str | None = None,          # ← NEW: optional path to WingLoop_Library or similar
        Dt: float = 0.01,
    ):
        """
        Args:
            control_directory:      Folder containing startup_file, control_file, precomputed_file
                                    (recommended: absolute path or relative to cwd)
            startup_file:           Name of startup script (e.g. 'startup.m', 'compute_gains.py')
            control_file:           Main controller file (determines method): .m / .py / .slx / .fmu
            precomputed_file:       Optional name of precomputed data file (relative to control_directory)
            library_path:           Optional path to additional MATLAB/Python library folder
                                    (e.g. WingLoop_Library containing functions/FMUs)
            Dt:                     Time step for controller
        """
        # Normalize and make control_directory absolute → safer
        self.control_directory = os.path.abspath(os.path.normpath(control_directory))
        if not os.path.isdir(self.control_directory):
            raise NotADirectoryError(f"control_directory does not exist or is not a folder: {self.control_directory}")

        self.startup_file = startup_file
        self.control_file = control_file
        self.precomputed_file = precomputed_file
        self.Dt = Dt

        # Full startup paths (relative → absolute resolution)
        self.startup_path = os.path.join(self.control_directory, startup_file) if startup_file else None
        if self.startup_path and not os.path.isfile(self.startup_path):
            raise FileNotFoundError(f"Startup file not found: {self.startup_path}")
        self.control_path = os.path.join(self.control_directory, control_file)

        # Full control paths (relative → absolute resolution)
        if not os.path.isfile(self.control_path):
            raise FileNotFoundError(f"Control file not found: {self.control_path}")
        
        # Full precomputed paths (relative → absolute resolution)
        self.precomputed_path = os.path.join(self.control_directory, precomputed_file) if precomputed_file else None
        if self.precomputed_path and not os.path.isfile(self.precomputed_path):
            warnings.warn(
                f"Precomputed file specified but not found → will compute instead.\n"
                f"Missing: {self.precomputed_path}",
                UserWarning
            )
            self.precomputed_path = None

        # Library / toolbox path (used for MATLAB addpath and FMU location)
        if library_path is None:
            # Fallback: try to guess common locations (customize this!)
            possible = [
                os.path.expanduser("~/Bureau/01 Github/02_WingLoop/WingLoop_Library"),
                os.path.expanduser("~/GitHub/02_WingLoop/WingLoop_Library"),
                os.getcwd(),  # last resort
            ]
            for p in possible:
                if os.path.isdir(p):
                    library_path = p
                    break
            else:
                library_path = self.control_directory  # fallback
        self.library_path = os.path.abspath(library_path)

        # Determine method
        ext = os.path.splitext(control_file)[1].lower()
        method_map = {
            '.py':  "python",
            '.m':   "matlab",
            '.slx': "simulink",
            '.fmu': "simulink_fmu",
        }
        if ext not in method_map:
            raise ValueError(f"Unsupported control_file extension: {ext} (file: {control_file})")
        self.method = method_map[ext]

        # Store control function/model name for later use
        self.control_function_or_model = os.path.splitext(control_file)[0]

        self.eng = None
        self.time = 0.0
        self.fmu_controller = None
        self.python_controller_instance = None  # For python class instance
        self.matlab_controller_instance = None  # For matlab class instance

        # MATLAB engine setup for MATLAB-related methods
        if self.method in ["matlab", "simulink", "simulink_fmu"]:
            self.eng = matlab.engine.start_matlab("-nodesktop -nosplash -nojvm")  # -nojvm often faster

            # 1. Change directory → most reliable
            self.eng.cd(self.control_directory, nargout=0)

            # 2. Add library path (utilities, functions, etc.)
            self.eng.addpath(self.library_path, nargout=0)

        # Simulink model setup
        if self.method == "simulink":
            try:
                self.eng.load_system(self.control_function_or_model, nargout=0)
                self.eng.set_param(self.control_function_or_model, 'ReturnWorkspaceOutputs', 'on', nargout=0)
                self.eng.set_param(self.control_function_or_model, 'SignalLogging', 'on', nargout=0)
            except Exception as e:
                raise RuntimeError(f"Failed to load/configure Simulink model '{self.control_function_or_model}': {e}")

        # FMU setup
        elif self.method == "simulink_fmu":
            # FMU is usually in library_path or control_directory
            possible_fmu_locations = [
                os.path.join(self.library_path, control_file),
                os.path.join(self.control_directory, control_file),
                control_file,  # if absolute
            ]
            fmu_path = None
            for cand in possible_fmu_locations:
                if os.path.isfile(cand):
                    fmu_path = cand
                    break
            if fmu_path is None:
                raise FileNotFoundError(f"FMU not found. Tried:\n" + "\n".join(possible_fmu_locations))

            self.fmu_controller = SimulinkFMUController(
                fmu_path=fmu_path,
                Dt=self.Dt
            )

        # Run setup
        self.setup()

    def setup(self):
        """
        Handles step 1 (compute) or step 2 (load precomputed).
        Makes variables accessible for step 3 (simulation).
        """
        if self.method == "python":
            sys.path.append(self.control_directory)
            control_mod_name = os.path.splitext(self.control_file)[0]
            control_mod = import_module(control_mod_name)

            # Convention: the class must be named UserController
            # (you can make this configurable later if needed)
            try:
                UserControllerClass = getattr(control_mod, 'UserController')
            except AttributeError:
                raise AttributeError(
                    f"Python control file '{self.control_file}' must define a class named 'UserController'"
                )

            # Instantiate — pass paths so the class decides what to do
            self.python_controller_instance = UserControllerClass(
                precomputed_file_path = self.precomputed_path,  # full path or None
            )

        elif self.method in ["matlab", "simulink", "simulink_fmu"]:
            # Instantiate MATLAB UserController class — it handles precomputed or default compute
            precomp_path_mat = self.precomputed_path or ""
            self.matlab_controller_instance = self.eng.UserController(precomp_path_mat)
            # For simulink, set to base workspace
            #if self.method == "simulink":
            #    self.eng.feval('setToBaseWorkspace', self.matlab_controller_instance)
            ## For fmu, get data and set on FMU
            #elif self.method == "simulink_fmu":
            #    #data_ml = self.eng.feval('getInitialData', self.matlab_controller_instance, nargout=1)
            #    data_ml = self.eng.UserController(precomp_path_mat)
            #    # Convert matlab.struct to Python dict
            #    data = {k: np.array(v) for k, v in data_ml.items()}
            #    param_vrs = {v.name: v.valueReference for v in self.fmu_controller.model_description.modelVariables if v.causality == 'parameter'}
            #    for key, value in data.items():
            #        if key in param_vrs:
            #            self.fmu_controller.fmu.setReal([param_vrs[key]], value.flatten().tolist())

    def PyControl_DoControllerStep(self, instantaneous_state, Dt):
        if self.method=="matlab":
            state_ml = matlab.double(instantaneous_state.tolist())  # 1×N or Nx1
            out_ml = self.eng.feval('step', self.matlab_controller_instance, state_ml, Dt, nargout=1)
            # out_ml is Python dict because MATLAB struct → dict
            output= {
                "F1": float(out_ml['F1']),
                "F2": float(out_ml['F2']),
                "F3": float(out_ml['F3']),
                "F4": float(out_ml['F4']),
                "E1": float(out_ml['E1']),
                "E2": float(out_ml['E2'])
            }
        elif self.method=="simulink_fmu":
            output = self.fmu_controller.step(instantaneous_state)

        elif self.method == "simulink":
            state_array = np.array(instantaneous_state, dtype=np.float64).flatten()
            if len(state_array) != 1945:
                raise ValueError("State vector must be 1945 long")
                       
            t_vec = [self.time, self.time + self.Dt]
            u_mat = [state_array.tolist(), state_array.tolist()]

            external_input = matlab.double(
                [[t_vec[0]] + u_mat[0],
                [t_vec[1]] + u_mat[1]]
            )

            self.eng.workspace['statein'] = external_input

            # Short simulation
            self.eng.workspace['Tstart'] = self.time
            self.eng.workspace['Tstop']  = self.time + self.Dt

            out = self.eng.sim(
                self.control_function_or_model,
                'StartTime', 'Tstart',
                'StopTime',  'Tstop',
                'LoadExternalInput', 'on',
                'ExternalInput', 'statein',
                nargout=1
            )

            # === CRITICAL: Push the SimulationOutput to MATLAB workspace ===
            self.eng.workspace['simOut'] = out

            # Debug (run once, then you can comment out)
            # print("Fields in simOut:", self.eng.eval("fieldnames(simOut)", nargout=1))

            # Robust extraction — works whether 1 sample or many
            try:
                output = {
                    "F1": float(self.eng.eval("simOut.F1.Data(end)", nargout=1)),
                    "F2": float(self.eng.eval("simOut.F2.Data(end)", nargout=1)),
                    "F3": float(self.eng.eval("simOut.F3.Data(end)", nargout=1)),
                    "F4": float(self.eng.eval("simOut.F4.Data(end)", nargout=1)),
                    "E1": float(self.eng.eval("simOut.E1.Data(end)", nargout=1)),
                    "E2": float(self.eng.eval("simOut.E2.Data(end)", nargout=1)),
                }
                # print("✅ Success — Control outputs:", output)   # remove after it works

            except Exception as e:
                print("Extract failed (trying Array format fallback):", str(e))
                # Fallback if someone accidentally set Save format = Array
                output = {
                    "F1": float(self.eng.eval("simOut.F1(end)", nargout=1)),
                    "F2": float(self.eng.eval("simOut.F2(end)", nargout=1)),
                    "F3": float(self.eng.eval("simOut.F3(end)", nargout=1)),
                    "F4": float(self.eng.eval("simOut.F4(end)", nargout=1)),
                    "E1": float(self.eng.eval("simOut.E1(end)", nargout=1)),
                    "E2": float(self.eng.eval("simOut.E2(end)", nargout=1)),
                }

            self.time += self.Dt

        if self.method == "python":
            if self.python_controller_instance is None:
                raise RuntimeError("Python controller instance not initialized")
            output = self.python_controller_instance.step(instantaneous_state, Dt)
        #else:
        #    pass
            # ... other methods are possible ...
        print(output)
        return output


class SimulinkFMUController:
    def __init__(self, fmu_path, Dt=0.01):
        """
        fmu_path: full path to your .fmu file (string)
        """
        self.fmu_path = fmu_path
        self.Dt = Dt
        self.time = 0.0
        self.unzipdir = None
        self.fmu = None

        # Parse the model description (returns ModelDescription object)
        self.model_description = read_model_description(self.fmu_path)

        # Extract FMU contents to a temp directory (returns the unzip path string)
        self.unzipdir = extract(self.fmu_path)  # auto temp dir if not specified

        # Instantiate correctly:
        # 1st arg: unzip directory (string)
        # 2nd arg: model_description (ModelDescription object, NOT string!)
        self.fmu = instantiate_fmu(
            unzipdir=self.unzipdir,
            model_description=self.model_description,
            visible=False,
            debug_logging=False  # change to True for FMI debug output during dev
        )


    def step(self, instantaneous_state):
        state_array = np.array(instantaneous_state, dtype=np.float64).flatten()

        # Get all input variables for InputState[1..1945]
        input_vars = [v for v in self.model_description.modelVariables if v.name.startswith('statein[')]
        input_vars.sort(key=lambda v: v.valueReference)  # Ensure correct order

        if len(state_array) != len(input_vars):
            raise ValueError(f"State vector length ({len(state_array)}) does not match FMU input count ({len(input_vars)})")

        input_vrs = [v.valueReference for v in input_vars]

        # Set all 1945 inputs at once
        self.fmu.setReal(input_vrs, state_array.tolist())

        # Advance FMU
        self.fmu.doStep(currentCommunicationPoint=self.time, communicationStepSize=self.Dt)
        self.time += self.Dt

        # Get outputs
        output_names = ['F1', 'F2', 'F3', 'F4', 'E1', 'E2']
        output_vrs = [next((v.valueReference for v in self.model_description.modelVariables if v.name == n), None)
                    for n in output_names]
        output_values = self.fmu.getReal(output_vrs)
        return dict(zip(output_names, [float(v) for v in output_values]))

    def terminate(self):
        """Call this when done (e.g. in WingLoop cleanup) to free resources"""
        if self.fmu:
            try:
                self.fmu.terminate()
            except:
                pass
        if self.unzipdir and os.path.exists(self.unzipdir):
            shutil.rmtree(self.unzipdir, ignore_errors=True)



if __name__=="__main__":

    method = "simulink_fmu"
    IsPrecomputed = False

    # test_instantaneous_state is [10,11,12,13,...,19440,19450]
    # test_instantaneous_state has shape (1945,)
    test_instantaneous_state = ((np.arange(1945) + 1) * 10).astype(float)
    test_Dt = 0.01
    
    if not IsPrecomputed:
        precomputed_filename = None
    elif method == "python":
        precomputed_filename = "precomputed_python.npz"
    else:
        precomputed_filename = "precomputed.mat"

    if method == "python":
        ControlInstance = PyControl(
            control_directory="test_files/test_controllers/python",
            startup_file=None,
            control_file="python_test_controller.py",               # ← determines python method
            precomputed_file= precomputed_filename, # optional
            Dt=test_Dt
        )

    elif method == "matlab":
        ControlInstance = PyControl(
            control_directory="test_files/test_controllers/matlab/",
            startup_file=None,
            control_file="UserController.m",               # can be almost anything .m
            precomputed_file=precomputed_filename,
            Dt=test_Dt
        )
    elif method == "simulink":

        ControlInstance = PyControl(
            control_directory="test_files/test_controllers/simulink/",
            startup_file=None,
            control_file="simulink_test_controller.slx",    # ← determines simulink method
            precomputed_file=precomputed_filename,   # optional
            Dt=test_Dt
        )

    elif method == "simulink_fmu":

        ControlInstance = PyControl(
            control_directory="test_files/test_controllers/simulink_fmu/",
            startup_file=None,
            control_file="simulink_test_controller.fmu",    # ← determines simulink method
            precomputed_file=precomputed_filename,
            Dt=test_Dt
        )

    ControlInstance.PyControl_DoControllerStep(test_instantaneous_state, Dt=test_Dt)