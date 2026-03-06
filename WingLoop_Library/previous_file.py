"""
====================================================================================
PREVIOUS FILE

Author: Leonardo AVONI
Date: 08/11/2024
Email: avonileonardo@gmail.com

Last modified: 29/05/2025

====================================================================================

Description:
This Python class provides the following
    storage space for timeseries describing the aircraft flight
    storage space for control laws
    
The function used by WingLoop for control is PyControl.UAV_control_Strategy(..). 
The other functions may be used by that method to help define control laws

====================================================================================
"""
import numpy as np
import matplotlib.pyplot as plt
import os
import matlab.engine
import fmpy
import os
import shutil
from fmpy import read_model_description, extract, instantiate_fmu

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

        # Optional: set initial parameters if your FMU has tunable ones
        # fmpy.set(self.fmu, 'your_param_name', some_value)
    def stepold(self, instantaneous_state):
        state_array = np.array(instantaneous_state, dtype=np.float64).flatten()

        # Collect all FMU input VRs
        # Get the FMU input variable corresponding to the Bus element
        print(self.model_description.modelVariables )
        input_var = next((v for v in self.model_description.modelVariables if 'InBus' in v.name), None)
        if input_var is None:
            raise ValueError("Bus input variable not found in FMU model description")


        # Optionally check size
        if len(state_array) != 1945:  # or len(input_var.start)
            raise ValueError(f"State vector length ({len(state_array)}) does not match FMU input size (1945)")

        # Set the vector input (single VR)
        self.fmu.setReal([input_var.valueReference], [state_array.tolist()])

        # Advance FMU
        self.fmu.doStep(currentCommunicationPoint=self.time, communicationStepSize=self.Dt)
        self.time += self.Dt

        # Get outputs
        output_names = ['F1', 'F2', 'F3', 'F4', 'E1', 'E2']
        output_vrs = [next((v.valueReference for v in self.model_description.modelVariables if v.name == n), None)
                    for n in output_names]
        output_values = self.fmu.getReal(output_vrs)
        return dict(zip(output_names, [float(v) for v in output_values]))

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


class PyControl:
    
    """
    Control class, containing the various control laws and history tracking of the variables used in Python
    """
    def __init__(self,method):
        

        self.eng = None
        self.method = method
        self.Dt = 0.01
        self.time = 0.0
        
        path = "/home/leonardo-avoni/Desktop/01_GitHub/02_WingLoop/WingLoop_Library"
        #path = "/home/daep/l.avoni/Bureau/01 Github/02_WingLoop/WingLoop_Library"
        path_simulink = os.path.join(path, "test_files/test_controllers/simulink/")
        path_fmu = os.path.join(path, "test_files/test_controllers/simulink_fmu/simulink_test_controller_old.fmu")

        self.eng = matlab.engine.start_matlab("-nodesktop -nosplash")

        if self.method == "simulink":
            # self.eng.set_param('UAV_Controller', 'SimulationCommand', 'step', nargout=0)
            self.eng.addpath(path_simulink, nargout=0)
            self.eng.load_system('simulink_test_controller_old')
            self.eng.set_param('simulink_test_controller_old', 'ReturnWorkspaceOutputs', 'on', nargout=0)
            #self.eng.set_param('UAV_Controller', 'SimulationCommand', 'start', nargout=0)
            #self.eng.set_param('UAV_Controller', 'SimulationCommand', 'pause', nargout=0)
            self.eng.set_param('simulink_test_controller_old', 'SignalLogging', 'on', nargout=0)

        elif self.method == "simulink_fmu":
            self.fmu_controller = SimulinkFMUController(
                fmu_path=path_fmu,
                Dt=0.01
            )

    
    
    
    def step(self,instantaneous_state):
        """ 
        Made for LQR, for Murua, trimmed
        
        the correct LQR equation, including for trimming point is: utot = -K(xtot-xt) + ut
        """
       
        # one could also use self.x_state_trimmed
        # since it's just a matrix multiplication (linear) we decide to 
        # precompute q_state_trimmed, so we only have to subtract length 32 vectors
        #q = self.W_T_M@instantaneous_state

        if self.method=="simulink_fmu":
            output = self.fmu_controller.step(instantaneous_state)

        elif self.method == "simulink":
            state_array = np.array(instantaneous_state, dtype=np.float64).flatten()
                        
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
                'simulink_test_controller_old',
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

        print(output)
        return output
    

method = "simulink_fmu"
instance = PyControl(method)
test_instantaneous_state = ((np.arange(1945) + 1) * 10).astype(float)
instance.step(test_instantaneous_state)