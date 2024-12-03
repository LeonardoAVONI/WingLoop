"""
====================================================================================
Control_Library, Version 2

Author: Leonardo AVONI
Date: 08/11/2024
Email: avonileonardo@gmail.com

Last modified: 08/11/2024

====================================================================================

Description:
This Python class provides the following
    storage space for timeseries describing the aircraft flight
    storage space for control laws

Features:
-append_flight_data: function made to append the flight data timestep after timestep
-PID: PID controller
-output_formatting: allows to link the physical model needs (which flap) to the control (which control)
-plot_the_data: allows to plot the timeseries

Comments:
-

Changelog:
- Version 1 (08/11/2024): creation of the code
- Version 2 (03/12/2024): clean version of the code, for Github


====================================================================================
"""
import numpy as np
import matplotlib.pyplot as plt

from scipy.integrate import trapezoid

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        # Initialize PID gains (as adimensional gains K's)
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Initialize PID state variables
        self.Previous_D_output = 0
        self.Previous_I_output = 0
        self.Previous_Error = 0
        self.Current_Error = 0

    def runPID(self,measured_variable,setpoint_variable,Ts):
        
        if False: #windup?
            if np.abs(self.Previous_I_output)>10:
                self.Previous_I_output = self.Previous_I_output*100/(np.abs(self.Previous_I_output))
            #print(self.Previous_I_output)
        # computing current error
        self.Current_Error = setpoint_variable - measured_variable

        # proportional term
        Out_P = self.Kp*self.Current_Error

        # derivative term
        #Out_D = -self.Previous_D_output + self.Kd*(1/Ts)*2*self.Previous_Error - self.Kd*(1/Ts)*2*self.Current_Error #trapezoidal,maybe a minus sign? WRONG
        #Out_D = self.Kd*(1/Ts)*self.Previous_Error - self.Kd*(1/Ts)*self.Current_Error #forward #maybe there should be a minus here WRONG
        Out_D = self.Kd*(1/Ts)*self.Current_Error - self.Kd*(1/Ts)*self.Previous_Error #from ChatGPT

        # integral term
        #Out_I = self.Previous_I_output - self.Ki*Ts*0.5*(self.Current_Error+self.Previous_Error) #trapezoidal WRONG
        #Out_I = self.Previous_I_output - self.Ki*Ts*self.Current_Error #forward WRONG
        #Out_I = self.Previous_I_output - self.Ki*Ts*self.Current_Error #backward WRONG
        Out_I = self.Previous_I_output + self.Ki * Ts * 0.5 * (self.Current_Error + self.Previous_Error) #chatgpt
        # summing up the contributions
        Out=Out_P+Out_D+Out_I

        # updating the variables value
        self.Previous_D_output = Out_D
        self.Previous_I_output = Out_I
        self.Previous_Error = self.Current_Error

        return Out

class Control:
    def __init__(self,K,Ti,Td):
        
        """
        Control calss, containing the various control laws and history tracking of the variables used in Python
        """

        # Dynamic Tracking Variables
        self.TIME = None

        self.EarthX = None
        self.EarthY = None
        self.EarthZ = None

        self.PITCH = None
        self.ROLL = None
        self.YAW = None

        self.ALPHA = None
        self.BETA = None
        self.VELOCITY = None

        # Global variables specific to the control strategy: we define one instance of the PID class
        self.Alt_Contr = PIDController(Kp=K, Ki=K/Ti, Kd=K*Td) #K/Ti


    def append_flight_data(self, instantaneous_struct):
        
        """ 
        This function allows to append data available from the text file provided via ASWING to local variables in Python
        thus creating a time-history of the variables, available in Python
        
        Note that the time discretization used is Ts*K (sampling time used by aswing * number of aswing iterations per python iteration)
        """
        # Check if the arrays are initialized

        if self.TIME is None:
            # If they aren't, initialize them
            self.TIME = np.array([instantaneous_struct["Time"]])

            self.PITCH = np.array([instantaneous_struct["Pitch"]])
            self.ROLL = np.array([instantaneous_struct["Bank"]])
            self.YAW = np.array([instantaneous_struct["Heading"]])

            self.EarthX = np.array([instantaneous_struct["earthX"]])
            self.EarthY = np.array([instantaneous_struct["earthY"]])
            self.EarthZ = np.array([instantaneous_struct["earthZ"]])

            self.ALPHA = np.array([instantaneous_struct["Alpha"]])
            self.BETA = np.array([instantaneous_struct["Beta"]])
            self.VELOCITY = np.array([instantaneous_struct["Velocity"]])

            self.F2 = np.array([instantaneous_struct["F2"]])

        else:
            # If they are initialized, append the new values
            self.TIME = np.append(self.TIME, instantaneous_struct["Time"])

            self.PITCH = np.append(self.PITCH, instantaneous_struct["Pitch"])
            self.ROLL = np.append(self.ROLL, instantaneous_struct["Bank"])
            self.YAW = np.append(self.YAW, instantaneous_struct["Heading"])

            self.EarthX = np.append(self.EarthX, instantaneous_struct["earthX"])
            self.EarthY = np.append(self.EarthY, instantaneous_struct["earthY"])
            self.EarthZ = np.append(self.EarthZ, instantaneous_struct["earthZ"])

            self.ALPHA = np.append(self.ALPHA, instantaneous_struct["Alpha"])
            self.BETA = np.append(self.BETA, instantaneous_struct["Beta"])
            self.VELOCITY = np.append(self.VELOCITY, instantaneous_struct["Velocity"])

            self.F2 = np.append(self.F2, instantaneous_struct["F2"])

    
    def UAV_control_Strategy(self,instantaneous_flight_data, Dt):
        """
        Example of a control law used for the current UAV:
            PID in pitch tracking (using a PID) and fixed velocity
            Limitations in possible aileron deflections
        """

        command_data = {}
        command_data["V"]=30

        selection = "V0"

        if selection == "V0":
            command_data["E1"]=7.559
            command_data["Pitch"]=0.5706
        if selection == "V2":
            command_data["E1"]=7.573
            command_data["Pitch"]=0.5067

        current_time=instantaneous_flight_data["Time"]
        #if current_time>0:
        #    command_data["Pitch"]=5

        Elevator_out = self.Alt_Contr.runPID(instantaneous_flight_data["Pitch"],command_data["Pitch"],Dt)

        # Sending the final instructions
        output = {}
        output["F1"]= 0
        output["F2"]= -Elevator_out #increasing F2 decreases pitch, hence the minus sign command_data["dF2"]
        output["F3"]= 0
        output["F4"]= 0
        
        # forcing the engine output
        output["E1"]=command_data["E1"]

        if False: #forced step response
            if current_time<0.1:
                output["F2"]=-10 #will pitch up

        # regulating max and min elevator deflection (avoid elevator stall, and keep results useable)
        if output["F2"]>10:
            output["F2"]=10
        if output["F2"]<-10:
            output["F2"]=10

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









