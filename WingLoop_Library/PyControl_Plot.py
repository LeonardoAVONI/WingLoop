import numpy as np

We need to make somethinf PyControl-compatible, to plot some data

    

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
