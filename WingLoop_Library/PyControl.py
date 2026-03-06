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
        save_precomputed_after_compute: bool = False,
        library_path: str | None = None,          # ← NEW: optional path to WingLoop_Library or similar
    ):
        """
        Args:
            control_directory:      Folder containing startup_file, control_file, precomputed_file
                                    (recommended: absolute path or relative to cwd)
            startup_file:           Name of startup script (e.g. 'startup.m', 'compute_gains.py')
            control_file:           Main controller file (determines method): .m / .py / .slx / .fmu
            precomputed_file:       Optional name of precomputed data file (relative to control_directory)
            save_precomputed_after_compute:
                                    If True and precomputed_file=None, auto-save after computation
            library_path:           Optional path to additional MATLAB/Python library folder
                                    (e.g. WingLoop_Library containing functions/FMUs)
        """
        # Normalize and make control_directory absolute → safer
        self.control_directory = os.path.abspath(os.path.normpath(control_directory))
        if not os.path.isdir(self.control_directory):
            raise NotADirectoryError(f"control_directory does not exist or is not a folder: {self.control_directory}")

        self.startup_file = startup_file
        self.control_file = control_file
        self.precomputed_file = precomputed_file
        self.save_precomputed_after_compute = save_precomputed_after_compute

        # Full paths (relative → absolute resolution)
        self.startup_path = os.path.join(self.control_directory, startup_file)
        self.control_path = os.path.join(self.control_directory, control_file)

        if not os.path.isfile(self.startup_path):
            raise FileNotFoundError(f"Startup file not found: {self.startup_path}")
        if not os.path.isfile(self.control_path):
            raise FileNotFoundError(f"Control file not found: {self.control_path}")

        if precomputed_file is not None:
            self.precomputed_path = os.path.join(self.control_directory, precomputed_file)
            if not os.path.isfile(self.precomputed_path):
                warnings.warn(
                    f"Precomputed file specified but not found → will compute instead.\n"
                    f"Missing: {self.precomputed_path}",
                    UserWarning
                )
                self.precomputed_path = None
        else:
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

        self.eng = None
        self.Dt = 0.01
        self.time = 0.0
        self.fmu_controller = None

        # MATLAB engine setup
        if self.method in ["matlab", "simulink"]:
            import matlab.engine
            self.eng = matlab.engine.start_matlab("-nodesktop -nosplash -nojvm")  # -nojvm often faster

            # 1. Change directory → most reliable
            self.eng.cd(self.control_directory, nargout=0)

            # 2. Add library path (utilities, functions, etc.)
            self.eng.addpath(self.library_path, nargout=0)

            # Optional: add genpath() if you have many subfolders
            # self.eng.addpath(self.eng.genpath(self.library_path), nargout=0)
            
            
        # Simulink model name (without .slx)
        if self.method == "simulink":
            print("fcwececec TEST")
            model_name = os.path.splitext(control_file)[0]

            try:
                self.eng.load_system(model_name, nargout=0)
                self.eng.set_param(model_name, 'ReturnWorkspaceOutputs', 'on', nargout=0)
                self.eng.set_param(model_name, 'SignalLogging', 'on', nargout=0)
            except Exception as e:
                raise RuntimeError(f"Failed to load/configure Simulink model '{model_name}': {e}")

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

            #from .SimulinkFMUController import SimulinkFMUController  # adjust import as needed
            self.fmu_controller = SimulinkFMUController(
                fmu_path=fmu_path,
                Dt=self.Dt
            )
            
        self.x_state_trimmed = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]

        # Run setup
        self.setup()

    def setup(self):
        """
        Handles step 1 (compute) or step 2 (load precomputed).
        Makes variables accessible for step 3 (simulation).
        """
        full_startup = os.path.join(self.control_directory, self.startup_file)
        use_precomputed = self.precomputed_file is not None
        full_precomputed = os.path.join(self.control_directory, self.precomputed_file) if use_precomputed else None

        if use_precomputed and not os.path.exists(full_precomputed):
            print(f"Precomputed file {full_precomputed} not found; falling back to computation.")
            use_precomputed = False

        if use_precomputed:
            # Load precomputed
            if self.method == "python":
                data = np.load(full_precomputed)
                self.K_x = data['K_x']
                self.W_T_M = data['W_T_M']
                #self.q_state_trimmed = data['q_state_trimmed']  # Or recompute if needed: self.W_T_M @ self.x_state_trimmed

            elif self.method in ["matlab", "simulink"]:
                self.eng.load(full_precomputed)

            elif self.method == "simulink_fmu":
                # Load from .mat or .npz and set as FMU parameters
                if full_precomputed.endswith('.mat'):
                    data = scipy.io.loadmat(full_precomputed)
                else:
                    data = np.load(full_precomputed)
                param_vrs = {v.name: v.valueReference for v in self.fmu_controller.model_description.modelVariables if v.causality == 'parameter'}
                for key, value in data.items():
                    if not key.startswith('__') and key in param_vrs:
                        # Assume scalar/array; adjust for your FMU (e.g., flatten if matrix)
                        self.fmu_controller.fmu.setReal([param_vrs[key]], value.flatten().tolist())

        else:
            # Compute using startup_file
            if self.method == "python" or self.method == "simulink_fmu":  # Python-based computation
                sys.path.append(self.control_directory)
                startup_mod_name = os.path.splitext(self.startup_file)[0]
                startup_mod = import_module(startup_mod_name)
                data = startup_mod.compute_initial_data(self.x_state_trimmed)  # Assume this function exists in startup_file
                self.K_x = data['K_x']
                self.W_T_M = data['W_T_M']
                #self.q_state_trimmed = data.get('q_state_trimmed', self.W_T_M @ self.x_state_trimmed)

            elif self.method in ["matlab", "simulink"]:
                self.eng.cd(self.control_directory)
                # Pass trimmed state to workspace for computation
                self.eng.workspace['x_state_trimmed'] = matlab.double(self.x_state_trimmed.tolist())
                self.eng.run(os.path.splitext(self.startup_file)[0])  # Runs the .m script

                # Fetch computed vars if needed (optional; they're in workspace for simulation)
                # self.K_x = np.array(self.eng.workspace['K_x'])  # If you need them in Python

            if self.save_precomputed_after_compute:
                # Save computed data for future use (default filename based on method)
                save_path = os.path.join(self.control_directory, f"precomputed_{self.method}.{'npz' if self.method in ['python', 'simulink_fmu'] else 'mat'}")
                if self.method == "python" or self.method == "simulink_fmu":
                    np.savez(save_path, K_x=self.K_x, W_T_M=self.W_T_M)#, q_state_trimmed=self.q_state_trimmed)
                elif self.method in ["matlab", "simulink"]:
                    self.eng.save(save_path)

    # Your existing UAV_control_Strategy_LQR and UAV_control_Strategy methods go here (unchanged)
    # They will use the setup variables appropriately (e.g., self.K_x for python, workspace for matlab/simulink)

    

    def UAV_control_Strategy_LQR(self,instantaneous_state, Dt):
        """ 
        Made for LQR, for Murua, trimmed
        
        the correct LQR equation, including for trimming point is: utot = -K(xtot-xt) + ut
        """
       
        # one could also use self.x_state_trimmed
        # since it's just a matrix multiplication (linear) we decide to 
        # precompute q_state_trimmed, so we only have to subtract length 32 vectors
        #q = self.W_T_M@instantaneous_state
        if self.method=="matlab":
            state_ml = matlab.double(instantaneous_state.tolist())  # 1×N or Nx1
            out_ml = self.eng.UAV_control_Strategy_LQR(state_ml, Dt, nargout=1)
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

            #self.eng.workspace['statein'] = matlab.double(state_array.tolist())

            # Short simulation
            self.eng.workspace['Tstart'] = self.time
            self.eng.workspace['Tstop']  = self.time + self.Dt

            out = self.eng.sim(
                'UAV_Controller',
                'StartTime', 'Tstart',
                'StopTime',  'Tstop',
                'LoadExternalInput', 'on',
                'ExternalInput', 'statein',
                nargout=1
            )

            # === CRITICAL: Push the SimulationOutput to MATLAB workspace ===
            self.eng.workspace['simOut'] = out

            # Debug (run once, then you can comment out)
            print("Fields in simOut:", self.eng.eval("fieldnames(simOut)", nargout=1))

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
                print("✅ Success — Control outputs:", output)   # remove after it works

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

        elif self.method=="python":
            # your existing PyControl call
            dx = instantaneous_state-self.x_state_trimmed
            du = self.trimmed_inputs -self.K_x@(dx)
            
            # with u = -K @ (q - q_trim) + u_trim
            # with K either K_r_from_full or K_r_from_reduced
            #du = self.trimmed_inputs - self.K_r_from_reduced@(q-self.q_state_trimmed)
            
            # Apply limits to du
            du[:4] = np.clip(du[:4], -20, 20)  # Limit du[0,1,2,3] between -20 and 20
            du[4:] = np.clip(du[4:], -10, 10)   # Limit du[4,5] between 0 and 10 #possible to make negative thrust
                    
            # Sending the final instructions
            output = {}
            output["F1"]=  du[0]
            output["F2"]=  du[1]
            output["F3"]=  du[2]
            output["F4"]= du[3]
            
            # forcing the engine output
            output["E1"]= du[4]
            output["E2"]= du[5]
        print(output)
        return output
    


    def UAV_control_Strategy(self,instantaneous_flight_data, Dt):
        """
        Example of a control law used for the current UAV:
            PID in pitch tracking (using a PID) and fixed velocity
            Limitations in possible aileron deflections  (pm 10 degrees)
        """

        print(instantaneous_flight_data)

        command_data = {}


        #flexible case data
        command_data["WRBM"]= 1926.26 # Nm
        command_data["F2"]= -4.73592 # degrees
        command_data["E1"]= 2.69574 # degrees
        #command_data["Pitch"]= 4.7572999000549316 # degrees
        command_data["Pitch"] = 4.75853 # degrees
        command_data["AoA"]= 4.7585043907165527
        command_data["Wx"] = 0
        command_data["Wy"] = 0
        command_data["Wz"] = 0
        command_data["Wdotx"] = 0
        command_data["Wdoty"] = 0
        command_data["Wdotz"] = 0
        command_data["Bank"] = 0
        command_data["Heading"] = 0
        
        
        current_time=instantaneous_flight_data["Time"]
        #if current_time>0:
        #    command_data["Pitch"]=5

        #Elevator_out = self.Alt_Contr.runPID(instantaneous_flight_data["Pitch"],command_data["Pitch"],Dt)
        """
        From experimental data:
            Aircraft pitching up leads to positive pitching
            Aircraft yawing left has a negative heading
            Aircraft rolling left has a negative roll
        """
        """
        aileron_gains = np.array([0.2,-7.5,0.01])
        elevator_gains = np.array([-3,-300,-0.02])
        rudder_gains = np.array([1,-100,0])
        print("test")
        
        aileron_command = aileron_gains[0]*(command_data["Wx"]-instantaneous_flight_data["Wx"])+aileron_gains[1]*(command_data["Bank"]-instantaneous_flight_data["Bank"]) + aileron_gains[2]*(command_data["Wdotx"]-instantaneous_flight_data["Wdotx"]) #p i d in degrees
        elevator_command = elevator_gains[0]*(command_data["Wy"]-instantaneous_flight_data["Wy"]) +elevator_gains[1]*(command_data["Pitch"]-instantaneous_flight_data["Pitch"]) +elevator_gains[2]*(command_data["Wdoty"]-instantaneous_flight_data["Wdoty"]) #p i d in degrees
        rudder_command =  rudder_gains[0]*(command_data["Wz"]-instantaneous_flight_data["Wz"]) +rudder_gains[1]*(command_data["Heading"]-instantaneous_flight_data["Heading"]) + rudder_gains[2]*(command_data["Wdotz"]-instantaneous_flight_data["Wdotz"]) #p i d in degrees
        """
    
        # AIAA Time Test + Validation againts ASWING
        elevator_command = self.PID_controller.runPID_continuousWy(instantaneous_flight_data["Pitch"],instantaneous_flight_data["Wy"],command_data["Pitch"],Ts=Dt)
        
        
        # Sending the final instructions
        output = {}
        output["F1"]= 0#aileron_command
        output["F2"]= command_data["F2"] - elevator_command[0] #increasing F2 decreases pitch, hence the minus sign command_data["dF2"]
        output["F3"]= 0#rudder_command
        output["F4"]= 0
        
        # forcing the engine output
        output["E1"]=command_data["E1"]
        output["E2"] = output["E1"]

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
    
    method = "simulink"
    if method == "python":
        ControlInstance = PyControl(
            control_directory="test_files/test_controllers/python",
            startup_file="python_test_startup.py",
            control_file="python_test_controller.py",               # ← determines python method
            precomputed_file="precomputed_python.npz",  # optional
            save_precomputed_after_compute=True
        )

        
    elif method == "matlab":
        ControlInstance = PyControl(
            control_directory="test_files/test_controllers/matlab/",
            startup_file="matlab_test_startup.m",
            control_file="matlab_test_controller.m",               # can be almost anything .m
            precomputed_file="precomputed.mat",
        )
    elif method == "simulink":
        
        ControlInstance = PyControl(
            control_directory="test_files/test_controllers/simulink/",
            startup_file="simulink_test_startup.m",
            control_file="simulink_test_controller.slx",    # ← determines simulink method
            precomputed_file="precomputed.mat",   # optional
            save_precomputed_after_compute=True
        )
    
    
    elif method == "simulink_fmu":
        
        ControlInstance = PyControl(
            control_directory="test_files/test_controllers/simulink_fmu/",
            startup_file="simulink_fmu_test_startup.py",
            control_file="simulink_test_controller.fmu",    # ← determines simulink method
            precomputed_file="precomputed.mat",   # optional
            save_precomputed_after_compute=True
        )
    