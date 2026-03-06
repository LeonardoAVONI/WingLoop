"""
====================================================================================
Control_Library, Package Version

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
#from WingLoop_Library.Text2Python_Library import extract_states_vector
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
    def __init__(self,method,Dt):
              

        self.x_state_trimmed = np.zeros(1945)

        self.eng = None
        self.method = method
        self.Dt = Dt
        self.time = 0.0
        
        #path = "/home/leonardo-avoni/Desktop/01_GitHub/02_WingLoop/WingLoop_Library"
        path = "/home/daep/l.avoni/Bureau/01 Github/02_WingLoop/WingLoop_Library"
        
        if self.method in ["matlab", "simulink"]:
            self.eng = matlab.engine.start_matlab("-nodesktop -nosplash")

        if self.method == "matlab":
            self.eng.addpath(os.path.join(path,"test_files/test_files/test_controllers/matlab"), nargout=0)
            self.eng.workspace['Dt'] = 0.01  # optional
        elif self.method == "simulink":
            # self.eng.set_param('UAV_Controller', 'SimulationCommand', 'step', nargout=0)
            self.eng.addpath(os.path.join(path,"test_files/test_files/test_controllers/simulink"), nargout=0)
            self.eng.load_system('simulink_test_controller')
            self.eng.set_param('simulink_test_controller', 'ReturnWorkspaceOutputs', 'on', nargout=0)
            #self.eng.set_param('UAV_Controller', 'SimulationCommand', 'start', nargout=0)
            #self.eng.set_param('UAV_Controller', 'SimulationCommand', 'pause', nargout=0)
            self.eng.set_param('simulink_test_controller', 'SignalLogging', 'on', nargout=0)

        elif self.method == "simulink_fmu":
            self.fmu_controller = SimulinkFMUController(
                fmu_path=os.path.join(path,"test_files/test_files/test_controllers/simulink_fmu/simulink_test_controller.fmu"),
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
            out_ml = self.eng.matlab_test_controller(state_ml, Dt, nargout=1)
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
                'simulink_test_controller',
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
            
            # "/home/daep/l.avoni/Bureau/01 Github/02_WingLoop/WingLoop_Library/test_files/test_controllers/python/python_test_controller,py"
            du = dx#self.trimmed_inputs -self.K_x@(dx)
            
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
    
    
    
    

    def plot_the_data(self):
        
        """
        Create a figure showing the timeseries evolution of
            Pitch, Roll, Yaw
            EarthX, EarthY, EarthZ
            Alpha, Beta
            Velocity (airspeed)
        over time
        
        A horizontal line is also placed to indicate the reference pitch
        """
        
        
        # Create a figure and a grid of subplots
        fig, axs = plt.subplots(4, 1, figsize=(10, 15))
        fig.suptitle('Various Data Over Time', fontsize=16)
        #print(self.TIME)
        # Plot PITCH, ROLL, YAW over TIME
        axs[0].plot(self.TIME, self.PITCH, label='Pitch', color='blue')
        axs[0].plot(self.TIME, self.ROLL, label='Roll', color='green')
        axs[0].plot(self.TIME, self.YAW, label='Yaw', color='red')
        axs[0].set_title('Pitch, Roll, Yaw Over Time')
        axs[0].set_xlabel('Time')
        axs[0].set_ylabel('Angle (degrees)')
        axs[0].legend()
        axs[0].grid(True)
        #axs[0].set_ylim([-10, 10])  # Set y-axis limits

        # Save to a .npy file (binary format)
        if False:
            name = "K_3_"
            np.save(name+"TIME.npy", self.TIME)
            np.save(name+"PITCH.npy", self.PITCH)
            np.save(name+"ALPHA.npy", self.ALPHA)
            np.save(name+"EarthZ.npy", self.EarthZ)
            np.save(name+"VELOCITY.npy", self.VELOCITY)


        # Add a horizontal line at pitch = 0.8877
        axs[0].axhline(y=0.5706, color='orange', linestyle='--', label='Command Pitch')
        axs[0].legend()  # Update legend to include the new line


        # Plot EarthX, EarthY, EarthZ over TIME
        axs[1].plot(self.TIME, self.EarthX, label='EarthX', color='blue')
        axs[1].plot(self.TIME, self.EarthY, label='EarthY', color='green')
        axs[1].plot(self.TIME, self.EarthZ, label='EarthZ', color='red')
        axs[1].set_title('EarthX, EarthY, EarthZ Over Time')
        axs[1].set_xlabel('Time')
        axs[1].set_ylabel('Position')
        axs[1].legend()
        axs[1].grid(True)
        axs[1].set_ylim([-3, 3])  # Set y-axis limits

        # Plot ALPHA, BETA over TIME
        axs[2].plot(self.TIME, self.ALPHA, label='Alpha', color='blue')
        axs[2].plot(self.TIME, self.BETA, label='Beta', color='green')
        axs[2].set_title('Alpha, Beta Over Time')
        axs[2].set_xlabel('Time')
        axs[2].set_ylabel('Angle (radians)')
        axs[2].legend()
        axs[2].grid(True)
        #axs[2].set_ylim([-10, 10])  # Set y-axis limits

        # Plot VELOCITY over TIME
        axs[3].plot(self.TIME, self.VELOCITY, label='Velocity', color='blue')
        axs[3].set_title('Velocity Over Time')
        axs[3].set_xlabel('Time')
        axs[3].set_ylabel('Velocity (units)')
        axs[3].legend()
        axs[3].grid(True)
        #axs[3].set_ylim([0, 40])  # Set y-axis limits

        # Adjust layout
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()

    def write_to_file(self):
        np.savez("00_python_data.npz", TIME=self.TIME, ALPHA=self.ALPHA, VELOCITY=self.VELOCITY, PITCH=self.PITCH, F2=self.F2, Integrator = self.Integrator)
    

    def obtain_the_metrics(self):
        
        """
        
        function made to compute, using available timeseries, the following values:
            settling_time
            rise_time
            peak_time
            peak_error
            overshoot
        the function then returns the result of an objective function (linear combination of some of those values) that has to be minimized to find the best PID
        
        """

        t=self.TIME
        y=self.PITCH

        reference_pitch=0.5706
        start_time=0.1

        # only analyzing from the PID start
        mask = t > start_time
        t = t[mask]
        y = y[mask]

        steady_state_value=reference_pitch

        peak_error = np.max(np.abs(y-steady_state_value))
        print("Absolute Peak Error:", peak_error)

        # Define thresholds as percentages of the peak value
        margin=0.01
        lower_threshold = (1-margin) * peak_error
        upper_threshold = (1+margin) * peak_error
        # Find the peak time to max absolute peak error
        peak_time_indices = np.where((np.abs(y-steady_state_value) >= lower_threshold) & (np.abs(y-steady_state_value) <= upper_threshold))[0]

        if len(peak_time_indices) > 0:
            peak_time = t[peak_time_indices[0]] - t[0] #take the first index that is true, and subtract to the first time from the step
        else:
            peak_time = None
        print("Peak time [s]:", peak_time) #peak time was checked


        # Find the rise time as first time at which error changes sign
        rise_time_indices = np.where(np.diff(np.sign(y-steady_state_value)) != 0)[0]
        if len(rise_time_indices) > 0:
            rise_time = t[rise_time_indices[0]] - t[0] #take the first index that is true, and subtract to the first time from the step
        else:
            rise_time = None
        print("Rise time [s]:", rise_time)


        # find the overshoot, also we want it in absolute value
        overshoot = (peak_error - steady_state_value) / steady_state_value * 100
        print("Overshoot (%):", overshoot)


        # Define a tolerance band around the steady-state value (e.g., 2%)
        tolerance = 0.2
        # Find the time when the signal stays within the tolerance band
        error = np.abs(y - steady_state_value)

        if error[-1] > tolerance: # the end point of the signal is larger than the tolerance
            settling_time=None
        else:
            buffer_error = error[::-1]
            buffer_time = t[::-1]
            settling_time_indices = np.where(buffer_error > tolerance)[0] # check where it is positive
            if not len(settling_time_indices): # depending on the treshold value all y-error value can be valid 
                settling_time=0
            else:
                settling_time = buffer_time[np.min(settling_time_indices)] - t[0] #pick the first item for which the condition holds

        print("Settling Time [s]:",settling_time)

        # evaluation

        a_rise=0.5
        a_peak=0.5
        a_set=1 #


        if settling_time==None or rise_time==None or peak_time==None:
            score = np.inf
        else:
            score = a_set*settling_time + a_rise*rise_time + a_peak*peak_error

        return score



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


if __name__=="__main__":
    
    method = "matlab"
    
    # test_instantaneous_state is [10,11,12,13,...,19440,19450]
    # test_instantaneous_state has shape (1945,)
    test_instantaneous_state = (np.arange(1945)+1)*10 
    test_Dt = 0.01
    
    ControlInstance = PyControl(method,test_Dt)
    
    ControlInstance.UAV_control_Strategy_LQR(test_instantaneous_state, Dt=test_Dt)    