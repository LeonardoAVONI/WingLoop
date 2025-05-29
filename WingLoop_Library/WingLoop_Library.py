"""
====================================================================================
WingLoop, Package Version

Author: Leonardo AVONI
Date: 25/10/2024
Email: avonileonardo@gmail.com

Last modified: 29/05/2025
====================================================================================

Description:
    This code is made for aeroservoelastic control of aircraft modeled using ASWING.
    It is meant as a way to extend ASWING's control capabilities using the toolboxes
    available in Python. It is also possible, although not implemented, to link Python
    with Matlab, thus implementing the methods available in Matlab.
    
    This Python code is made to be used with Aswing_Director library, used to send commands 
    to ASWING from Python. This code also uses functions from the following classes:
        Control_Library.PyControl: where all the timeseries and control laws to be used are stored
        Text2Python_Library: used to convert the python data to text data accessible by ASWING, 
            and vice-versa
    
    The WingLoop class comes with several methods:
        -Launch_ASWING: used to start ASWING with the appropriate parameters, like the file used and 
            the ASWING version used, and other parameters
        -Load_Files: used to load the various files needed by ASWING. The files are loaded by type according
            to their extension (.asw will be geometry, .state will be a state...)
        -Deactivate_Graphics: specific list of instructions for ASWING, to stop if from showing graphic 
            output. ASWING can still save images if needed, but those are not showed to the user
        -Time_Transient_Simulation: used to launch a time transient simulation. This function 
            calls Perform_K_iterations
        -Outputting_The_Results: plots and/or writes timeseries
        -Closing_WingLoop: does what it says

    More details are found in each function's description

How To Install
    Position yourself in the folder upstream of WingLoop_Library, i.e. the 
    folder containing this WingLoop_Library.py code, and the 
    Text2Python_Library.py, Control_Library.py codes.
    Alongside that folder there should be the pyproject.toml
    Open a terminal, and write "/usr/bin/python3 -m pip install --user -e ."
    This will install it in Python, in editable mode (if you modify the code it 
    will impact the package behavior)

====================================================================================
"""

import csv
import subprocess
import numpy as np
import sys
import os
import time
#import matlab.engine #executing this line steals a bit of time; it is needed if we want to bridge it with matlab
import threading
from queue import Queue, Empty
from ASWING_Director_Library import Aswing_Director
from WingLoop_Library.Text2Python_Library import text2python_main, python2text, text2python_withderivative
from WingLoop_Library.Control_Library import PyControl
import sys


class WingLoop:
    def __init__(self):
        """
        Declares the WingLoop class
        """
        pass
    
    def Launch_ASWING(self,aswing_path,sim_directory,asw_filename,print_output,
                      timer_text=0.000001,
                      finished_writing_check_timestep=0.001):
        """
        
        aswing_path (str): path at the end of which there's the ASWING executable to use
        sim_directory (str): local directory where the simulation files are
        asw_filename (str): .asw filename to use
        print_output (bool): if True, WingLoop data will be printed to the terminal
        timer_text (float): timer used by default for the Aswing_Director functions
        finished_writing_check_timestep (float): timer used to check for the last update on the size of the state file written by ASWING
        """
        # storing the variables to self.
        self.sim_directory = sim_directory
        self.print_output = print_output #used for the print output of the Aswing_Director
        self.count = 0 #used to count the number of iterations performed
        
        # record the location of the initial path
        self.initial_path = os.getcwd()
        # change path to go to the sim_directory path
        #os.chdir(os.path.join(self.initial_path,sim_directory))
        # create an ASWING instance
        self.ASW_handler = Aswing_Director(aswing_path=aswing_path,
                                           wait_time=timer_text, 
                                           finished_writing_file_check_timestep=finished_writing_check_timestep)
        self.ASW_handler.start_aswing(directory=sim_directory,filename=asw_filename,print_output=self.print_output)
        
        # create a Python Instance, for control reasons. It stores timeseries and control laws
        # the exact controller used is in UAV_control_Strategy (modifiable)
        self.PyControl = PyControl()
        
                
    def Load_Files(self,filename):
        """
        From the main ASWING menu, this function loads the file called "filename", 
        according to its extension. When using ASWING from the terminal, any file 
        extension can be loaded. Here the extension will make the difference

        Args:
            filename (string): filename to enter in ASWING from the main menu, including the extension
        """

        if filename.endswith(".asw"):
            stdout, stderr = self.ASW_handler.send_command_and_receive("load "+filename) #asw
        elif filename.endswith(".pnt"):
            stdout, stderr = self.ASW_handler.send_command_and_receive("pget "+filename) # pnt
        elif filename.endswith(".set"):
            stdout, stderr = self.ASW_handler.send_command_and_receive("sget "+filename) # set
        elif filename.endswith(".state"):
            stdout, stderr = self.ASW_handler.send_command_and_receive("tget "+filename) # state
        elif filename.endswith(".gust"):
            stdout, stderr = self.ASW_handler.send_command_and_receive("gget "+filename) # gust
        else:
            print("UNKNOWN FILENAME")
            sys.exit()
            
    def Deactivate_Graphics(self):
        """
        Defines the list of ASWING instructions needed to deactivate the graphical output of ASWING
        """
        stdout, stderr = self.ASW_handler.send_command_and_receive("PLPA")
        stdout, stderr = self.ASW_handler.send_command_and_receive("G")
        stdout, stderr = self.ASW_handler.send_command_and_receive("\n")
        
    def Performing_K_iterations_ASWING(self, Dt_aswing, K_aswing, custom_timer = None):

        """
        Works from the Oper menu, made to append K_aswing iterations with control to current simulation
        The function does the following:
            -writes current "output" state from ASWING to a text file
            -obtains from such state the (instantaneous) variables used for control
            -appends the instantaneous variables to the previous time series of previously appended values. 
                This operation can be done Python, but here it's done in Python
            -asking the PyControl.UAV_control_Strategy function for the controls needed at the next time step
            -writing the "input" text file to be used by ASWING for control (defining elevator inputs, engine input...)
            -send K_aswing iterations in ASWING, with Dt intervals with the generated "input" control text file

        """
        # we check if we are doing the very first time-transient simulation
        if self.count: 
            command=str(Dt_aswing)+" -"+str(K_aswing)
        else:
            command=str(Dt_aswing)+" "+str(K_aswing)
        # we define what the command is to perform K iterations using a certain 
        # input file called "input"
        x_command="x input"
        
        """
        Several options are available concerning the state file written by ASWING
            delete: this option deletes the older "output" file, so the new one has no problems being written
            overwrite: this method overwrites current "output" file
            none: this method does not write any file. It can be used for time testing
        
        note that time.sleep() are needed lower in the code if "none" is used
        """
        # the best option for time speed is overwrite
        state_file_write_options = "overwrite"
        
        if state_file_write_options != "none":
            if state_file_write_options == "delete":
                
                # This option deletes the previous input file before writing the new one                
                stdout, stderr = self.Aswing_handler.send_command_and_receive("W",custom_timer=custom_timer) # write the data
                os.remove("output")
                # Once the previous file is deleted, we can write the new one
                # There's no need for the "append_or_overwrite" option, since 
                # the previous file disappeared, so there's nothing to append 
                # "a" or overwrite "o"
                stdout, stderr , time_taken= self.Aswing_handler.send_writefile_command_and_receive(filename="output", 
                                                                                                    custom_timer=custom_timer)

            if state_file_write_options == "overwrite":
                #deletes the previously written data from the output file, it 
                # now will have a length 0 
                stdout, stderr = self.ASW_handler.send_command_and_receive("W",custom_timer=custom_timer) # write the data
                with open('output', 'w') as file: 
                    pass
                #checkig that the file is still there, but his content has been deleted
                while os.stat("output").st_size:
                    time.sleep(0.0000001)
                
                # The following method was checking the length modification of 
                # the previous file, to understand if it was done writing or no
                # However, THE OVERWRITE MODIFICATION, THAT RELIES ON CHECKING 
                # THE output FILE STATE (check if it was modified or not) ENDED 
                # UP NOT WORKING
                """
                    sys.exit()
                    #while os.stat("output").st_mtime == last_modification_time:
                    #    time.sleep(0.00001)
                    #last_modification_time = os.stat("output").st_mtime
                """
                # Now that we know the content of the previous input file was 
                # deleted, we can write the next one inside it
                stdout, stderr , time_taken= self.ASW_handler.send_writefile_command_and_receive(filename="output",  
                                                                                                    custom_timer=custom_timer,
                                                                                                    append_or_overwrite="O")
        
        # writing a dictionary containing the flight data specifics
        
        #instantaneous_flight_data = text2python_main("output")
        instantaneous_flight_data = text2python_withderivative("output")    
        
        # Appending the data in the python script
        self.PyControl.append_flight_data(instantaneous_flight_data)

        # Controlling through the python script
        output = self.PyControl.UAV_control_Strategy(instantaneous_flight_data, Dt = Dt_aswing*K_aswing)

        # create the file for the next iteration engine and flap deflections
        python2text("input",output)

        #perform K time iteration on aswing, using the input text file just written
        stdout, stderr = self.ASW_handler.send_command_and_receive(x_command, custom_timer=custom_timer) #create K_aswing iteration of the unsteady simulation
        stdout, stderr = self.ASW_handler.send_command_and_receive(command , custom_timer=custom_timer)

    def Define_Sim_Settings(self):
        """_summary_
        This function goes in the OPER menu, then settings, 
        then specifies some settings that need to be applied, 
        according to the input
        
        Would be nice to have it for automation
        """
        
        pass

    def Setting_Trimming_Point_Constraints():
        # Would be nice to have it for automation, but not defined at the moment
        # it would work using OPER>% not OPER>T
        pass

    def Time_Transient_Simulation(self,Dt,N,K):
        
        """ 
        Simulates the time-transient behavior of the aircraft starting from a pre-computed trimming point.
        This method performs a time-transient simulation in ASWING by iteratively solving the system dynamics.
        It begins by loading the necessary settings and importing a `.state` file containing the trimming point
        information. The `.state` file includes generally: constraints for free flight, fixed-stick trimmed control surface
        deflections, and engine power but is defaulted to "steady" simulation
        The simulation is divided into three phases:
        1. **Initial Iteration**: Performs the first set of L iterations starting from the trimming point. In general, L=K
        2. **Intermediate Iterations**: Loops through the simulation in chunks of `K` iterations until reaching just before
            the total number of iterations `N`.
        3. **Final Iteration**: Completes the remaining iterations to reach the total count `N`.
        Parameters:
            Dt (float): The time step for the simulation, in seconds
            N (int): The total number of iterations to perform.
            K (int): The number of iterations to perform in each chunk
        Notes:
            - The `.state` file must be created manually and loaded before running this simulation.
            - The method assumes that the ASWING handler (`self.ASW_handler`) is properly initialized and
                capable of sending commands to ASWING.
            - The `self.count` variable tracks the number of iterations already performed.
        Attributes:
            self.count (int): Tracks the number of iterations performed so far.
            self.print_output (bool): If True, prints progress information during the simulation.
        Workflow:
            - Switches ASWING to the `OPER` menu and sets the simulation mode to time-transient.
            - Performs the initial iteration using the trimming point.
            - Executes intermediate iterations in chunks of `K`.
            - Completes the final round of iterations to reach the total count `N`.
            - Optionally prints progress and outputs simulation data.
        """        

        # Go to OPER, time-transient simulation menu
        stdout, stderr = self.ASW_handler.send_command_and_receive("oper") #go to oper menu
        # Set the simulation to time-transient
        stdout, stderr = self.ASW_handler.send_command_and_receive(".") #switch to unsteady simulation

        # Note that self.count contains the number of iterations that have already been performed in ASWING
        # is self.count = 5, ASWING already performed 5 time iterations from the trimming point
        
        ### PERFORMING JUST THE FIRST ITERATION (starting from the pre-computed trimming point, saved in the .state file)
        #create a dummy, empty, output file
        if not os.path.exists("output"):
            open("output", "w")

        # it is possible to perform a smaller amount of initial simulations just for the very first step
        # here we set it to K (setting it to N is stupid, but possible)
        if K<N:
            L=K
        else:
            L=N
        # command for the first L time steps
        if L: #if L is non-zero, then L iterations are done with the aircraft in steady state, and flaps and engine fixed at steady-state positions
            command=str(Dt)+" "+str(L)
            stdout, stderr = self.ASW_handler.send_command_and_receive("x") #very first time-transient iteration
            stdout, stderr = self.ASW_handler.send_command_and_receive(command)
        self.count=L

        ### PERFORMING ALL INTERMEDIATE ITERATIONS (between L and N)
        while not (self.count + K >= N):
            # performing a number K of iteration (write output of the previous state, obtain the control command, send to aswing, perform K aswing iterations)
            if self.print_output:
                print("Internal iterations: from", self.count, " to ",self.count+K, " (step of ", K,")")
            self.Performing_K_iterations_ASWING(Dt_aswing=Dt, K_aswing=K)
            #incrementing the counter
            self.count += K

        ### PERFORMING THE FINAL ROUND OF ITERATIONS (between L+k*K and N)
        if self.count<N:
            if self.print_output:
                print("Final iterations: from", self.count, " to ",N, " (step of ", N - self.count,")")
            self.Performing_K_iterations_ASWING(Dt_aswing=Dt, K_aswing=N-self.count)
            self.count += N-self.count
        if self.print_output:
            print("Final Counter",self.count)
            
        ### Plotting The Data
        

### OUTPUT THE RESULTS

    def Outputting_The_Results(self,plot = True, timeseries=None):
        """_summary_

        Args:
            plot (bool, optional): do you want to show plots or not. Defaults to True.
            timeseries (string, optional): name of the timeseries to be saved. Defaults to None.
        """
        if plot:
            self.PyControl.plot_the_data()
        # writing an output timeseries for various variables
        if timeseries is not None:
            stdout, stderr = self.ASW_handler.send_command_and_receive("P")
            stdout, stderr = self.ASW_handler.send_command_and_receive("W")
            if os.path.exists(timeseries+".t"):
                os.remove(timeseries+".t")

            stdout, stderr = self.ASW_handler.send_command_and_receive(timeseries+".t")

    def Closing_WingLoop(self,removefiles = True):
        """Is in charge of closing the ASWING and ending the program

        Args:
            removefiles (bool, optional): If true, the input and output files for control are deleted. Defaults to True.
        """

        ### FINISHIING THE PROGRAM
        print("Ending WingLoop")
        time.sleep(0.5)
        # removing the communication files
        if removefiles:
            os.remove("output")
            os.remove("input")

        stdout, stderr = self.ASW_handler.send_command_and_receive("\n")  # go back to main menu
        # Quit Aswing
        self.ASW_handler.quit_and_close_aswing()


