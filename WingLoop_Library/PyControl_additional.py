

class PIDController:
    """ 
    Class defining a PID controller, defined with Kp, Ki and Kd
    
    The control instructions are then iterated in the time domain using discretization (Ts), written manually
    
    """
    
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
        #Out_I = self.Previous_I_output + self.Ki * Ts * 0.5 * (self.Current_Error + self.Previous_Error) #chatgpt
        Out_I = self.Previous_I_output + self.Ki * Ts*self.Current_Error # version for ASWING validation
        
        # summing up the contributions
        Out=Out_P+Out_D+Out_I

        # updating the variables value
        self.Previous_D_output = Out_D
        self.Previous_I_output = Out_I
        self.Previous_Error = self.Current_Error

        return Out

    def runPID_continuousWy(self,measured_variable,measured_derivative,setpoint_variable,Ts):
        
        self.Current_Error = setpoint_variable - measured_variable

        # proportional term
        Out_P = self.Kp*self.Current_Error

        # derivative term
        Out_D = self.Kd*(0-measured_derivative)

        # integral term
        Out_I = self.Previous_I_output + self.Ki * Ts*self.Current_Error # version for ASWING validation
        #Out_I = self.Previous_I_output + self.Ki * Ts * 0.5 * (self.Current_Error + self.Previous_Error) #chatgpt
        
        # summing up the contributions
        Out=Out_P+Out_D+Out_I

        # updating the variables value
        self.Previous_D_output = Out_D
        self.Previous_I_output = Out_I
        self.Previous_Error = self.Current_Error

        return Out , Out_I


 


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

