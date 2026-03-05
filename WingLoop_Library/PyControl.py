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
from WingLoop_Library.PyControl_Text2Python import extract_states_vector
from scipy.integrate import trapezoid
import matlab.engine
import fmpy
import os
import shutil
from fmpy import read_model_description, extract, instantiate_fmu


class PyControl:
    
    """
    Control class, containing the various control laws and history tracking of the variables used in Python
    """
    def __init__(self,control_directory,startup_file,control_file):
        

        self.x_state_trimmed = extract_states_vector("initial_state")
        self.trimmed_inputs = self.x_state_trimmed[-6:]
        
        #self.K_x = np.load('K_x.npy')
        #self.W_T_M = np.load('W_T_M.npy')
        
        #self.q_state_trimmed = self.W_T_M@self.x_state_trimmed #trimmed modal state
        #print(self.q_state_trimmed)
        # with u = -K @ (q - q_trim) + u_trim
        # with K either K_r_from_full or K_r_from_reduced
        # q = W_T_M@x

        self.eng = None
        self.method = "simulink"
        self.Dt = 0.01
        self.time = 0.0
        
        #path = "/home/leonardo-avoni/Desktop/01_GitHub/02_WingLoop/WingLoop_Library"
        path = "/home/daep/l.avoni/Bureau/01 Github/02_WingLoop/WingLoop_Library"
        fmupath = os.path.join(path, "UAV_Controller.fmu")

        if self.method in ["matlab", "simulink"]:
            self.eng = matlab.engine.start_matlab("-nodesktop -nosplash")

        if self.method == "matlab":
            self.eng.addpath(path, nargout=0)
            self.eng.workspace['Dt'] = 0.01  # optional
        elif self.method == "simulink":
            # self.eng.set_param('UAV_Controller', 'SimulationCommand', 'step', nargout=0)
            self.eng.addpath(path, nargout=0)
            self.eng.load_system('UAV_Controller')
            self.eng.set_param('UAV_Controller', 'ReturnWorkspaceOutputs', 'on', nargout=0)
            #self.eng.set_param('UAV_Controller', 'SimulationCommand', 'start', nargout=0)
            #self.eng.set_param('UAV_Controller', 'SimulationCommand', 'pause', nargout=0)
            self.eng.set_param('UAV_Controller', 'SignalLogging', 'on', nargout=0)

        elif self.method == "simulink_fmu":
            self.fmu_controller = SimulinkFMUController(
                fmu_path=fmupath,
                Dt=0.01
            )

    
    
    
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
    ControlInstance = PyControl(control_directory = "/test_files/test_controllers",
                                startup_file,
                                control_file)
    
""" 
I have here the following code. This code describes the PyControl class.

The idea is that at class startup, Pycontrol would require a directory where the control files would be located, as well as "the name of the control files"

The methods available are:
    python: if control files are .py files
    matlab: if control files are .m files
    simulink: if control files are .slx files
    simulink_fmu: if control files are .fmu files

The problem I have at the moment is: for control, I need to have "two" steps (regardless of whether I use python, matlab, simulink, r simulink_fmu):
    step 1: loading what is needed for the controller to use (control matrices, eigenvalues, requirements, weighhts matrices...) and perform some preliminary math to build what will be used by the controller
    step 2: as an option/alternative to step1, I would need to allow the user to load some pre-computed elements (if for example teh control matrices math was already doe, instead of recomputing them how about we just load them from a known file?)
    step 3: while the simulation is running timestep after timestep, the current time model state is fed to UAV_control_Strategy, that takes care of computing the controls to apply on the model. Depending on the method, the controller will be either  in matlab simulink or python

I need a way to provide to PyControl all of this. I need a way to make the thing general for Py, m, slx, fmu.
Take care that I need a way to make the pre-control variables accessible during the time-transient simulation

How can I do it?
"""