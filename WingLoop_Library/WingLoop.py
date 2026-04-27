# =============================================================================
# WingLoop
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
WingLoop - Aeroservoelastic Control Interface for ASWING
Version 2.0
====================================================================================

Author: Leonardo Avoni
Email: avonileonardo@gmail.com
Initial release: 25 Oct 2024
Last modified: 10 Mar 2026

------------------------------------------------------------------------------------
Overview
------------------------------------------------------------------------------------
WingLoop provides a Python interface for performing aeroservoelastic control
simulations of aircraft modeled in ASWING. The goal of the package is to extend
ASWING native control capabilities by leveraging the scientific computing and
control libraries available in Python.

The software communicates with ASWING through the Aswing_Director interface,
allowing Python scripts to send commands, retrieve simulation data, and apply
control laws during time-transient simulations.

For more information on how to use WingLoop, refer to the example case, 
stored in wingloop_testrun folder

------------------------------------------------------------------------------------
Dependencies
------------------------------------------------------------------------------------
WingLoop relies on the following internal modules:

• Aswing_Director
    Handles communication between Python and the ASWING executable.

• PyControl
    Contains control strategies, time-series storage, and controller execution.

• PyControl_IO / Text2Python utilities
    Convert data between Python structures and the text format required by ASWING.

• PyControl_Plot
    Provides real-time plotting and post-processing of simulation data.

------------------------------------------------------------------------------------
Main Class
------------------------------------------------------------------------------------
WingLoop

Primary responsibilities:
    • Launch and manage the ASWING simulation environment
    • Exchange state and control data between ASWING and Python/Matlab/Simulink control laws
    • Execute control laws during time-transient simulations
    • Record simulation time series
    • Provide live visualization and data export

------------------------------------------------------------------------------------
Main Methods
------------------------------------------------------------------------------------

Initialization / Setup
    Launch_ASWING()
    Launch_WingLoop_Control()
    InitializePlot()
    Load_Files()
    Deactivate_Graphics()

Simulation
    Time_Transient_Simulation()
    Performing_K_iterations_ASWING()

Output
    Outputting_The_State_File()
    Outputting_The_Results()

Shutdown
    Closing_WingLoop()

------------------------------------------------------------------------------------
Future Improvements
------------------------------------------------------------------------------------

Planned features:
    • Implement Setting_Trimming_Point_Constraints()
    • Implement Define_Sim_Settings()
    • Improve time synchronization between:
        - Python
        - MATLAB
        - Simulink
        - FMU simulations (time is different by Dt amount)

------------------------------------------------------------------------------------
Notes
------------------------------------------------------------------------------------
Trimming conditions, initial aircraft states, and simulation constraints must
currently be generated externally and loaded through a `.state` file before
running a transient simulation.

====================================================================================
"""

import numpy as np
import sys
import os
import time

#import Aswing_Director
#import PyControl
#from PyControl_Text2Python import python2text, initialize_data_dict, read_aswing_file, export_data_dict #import_data_dict
#from PyControl_Plot import ASWINGLivePlotter

from .Aswing_Director import *
from .PyControl import *
from .PyControl_Plot import *
from .PyControl_IO import *
from .PyControl_IO import _build_pattern
#from .PyControl_additional import *

class WingLoop:
    def __init__(self):
        """
        Declares the WingLoop class
        """
        pass
    
    def Launch_ASWING(self,aswing_fullpath,aswing_alias,sim_directory,asw_filename,print_output,
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
        self.ASW_handler = Aswing_Director(aswing_path=aswing_fullpath,aswing_alias= aswing_alias,
                                           wait_time=timer_text, 
                                           finished_writing_file_check_timestep=finished_writing_check_timestep)
        self.ASW_handler.start_aswing(directory=sim_directory,filename=asw_filename,print_output=self.print_output)


            
    def Launch_WingLoop_Control(self,cntrl_directory, 
                                cntrl_filename, 
                                timestep,
                                precomputed_filename = None, 
                                rebuild_fmu_file = False,
                                show_simulink_window = False):
        # create a Python Instance, for control reasons. It stores timeseries and control laws
        # the exact controller used is in UAV_control_Strategy (modifiable)
        if (cntrl_directory is not None) and (cntrl_filename is not None):
            self.PyControl = PyControl(
                control_directory = cntrl_directory, # where the controller is located
                control_file      = cntrl_filename, # which control file is used
                precomputed_file  = precomputed_filename, # which precomputed file is used (if none, )
                Dt                = timestep, #simulation timestep
                rebuild_fmu       = rebuild_fmu_file, # are we rebuilding the fmu file or not?
                show_simulink     = show_simulink_window, # are we showing the simulink window during execution?
            )
            self.fixed_stick = False
        else:
            self.fixed_stick = True


    def InitializePlot(self,liveplot,
                          plot_variables,
                          plot_sim_time,
                          plot_refreshtime,
                          plot_size = (16, 10),
                          N_steps = None):
        # this lists all available global variables we can plot
        self.LivePlot = liveplot
        requested_full = [

            # Position & Attitude
            "Time","earth X","earth Y","earth Z",
            "Heading","Elev.","Bank",
            "Alpha","Beta","Velocity",

            # Angular velocities / accelerations
            "Wx","Wy","Wz",
            "Wdotx","Wdoty","Wdotz",

            # Linear velocities / accelerations
            "Ux","Uy","Uz",
            "Udotx","Udoty","Udotz",

            # Moments
            "sum Mx","sum My","sum Mz",

            # Forces
            "sum Fx","sum Fy","sum Fz",

            # Aero & reference quantities
            "Lift","Density","Ref.Area",
            "Weight","Dyn.Pr.","Ref.Span",
            "Load Fac","VIAS","Ref.Chrd",
            "Mach","VTAS","MachPG",

            # Aero coefficients
            "CL","CD","L/D","Cl'",
            "Cm","CDi","e","Cn'",

            # Convergence (we will treat specially)
            "IsConverged",

            "Op.Point",
            "altitude"
        ]

        self.rename_map = {
    #        "earth X": "earthX",
    #        "earth Y": "earthY",
    #        "earth Z": "earthZ",
    #        "Elev.": "Pitch"
        }

        latex = {
    #        "Time": r"$t$",
    #        "earthX": r"$X_E$",
    #        "Velocity": r"$V$"
        }

        adimensional_vars = {
            "CL", "CD", "Cm", "Cn'",
            "Mach", "MachPG", "Load Fac","e"
        }

        # creates the required control elements: ["F1","F2",...,"F20","E1","E2",...,"E20"]
        # unused control elements will be set to "None"
        control_elements = [f"F{i}" for i in range(1, 21)] + [f"E{i}" for i in range(1, 21)]

        for ctrl in control_elements:
            if ctrl.startswith("F"):
                num = ctrl[1:]
                raw = f"Flap {num}"
            elif ctrl.startswith("E"):
                num = ctrl[1:]
                raw = f"Peng {num}"
            else:
                raise ValueError(f"Unknown control prefix in {ctrl}")
            requested_full.append(raw)
            self.rename_map[raw] = ctrl
            adimensional_vars.add(ctrl)

        # the PyCntrl_DATA dictionary will contain the timeseries of the 
        # used aircraft: global variables, controls and state vector
        # yes, PyCntrl_DATA is iterated at each timestep
        

        self.WingLoop_LogFile = initialize_data_dict(requested_full, 
                                                     self.rename_map, 
                                                     latex,
                                                     N_steps = N_steps)
        # pre-compile the parser pattern once
        self._compiled_pattern = _build_pattern(self.WingLoop_LogFile, self.rename_map)

        self.WingLoop_IsPlotting = False
        if not ((plot_variables == None) or (plot_sim_time == None) or (plot_refreshtime == None) or (plot_size == None) ):
            self.WingLoop_IsPlotting = True
        if self.WingLoop_IsPlotting:
            self.WingLoop_Liveplot = ASWINGLivePlotter(
                parameter_list = plot_variables,
                total_sim_time = plot_sim_time ,
                refresh_interval = plot_refreshtime,
                figsize = plot_size
            )
        self.writingtime = 0.0
        



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

        # allow for fixed-stick computations, with no control laws
        if self.fixed_stick:
            x_command ="x"
        
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
                stdout, stderr = self.ASW_handler.send_command_and_receive("W",custom_timer=custom_timer) # write the data
                os.remove("output")
                # Once the previous file is deleted, we can write the new one
                # There's no need for the "append_or_overwrite" option, since 
                # the previous file disappeared, so there's nothing to append 
                # "a" or overwrite "o"
                stdout, stderr , time_taken= self.ASW_handler.send_writefile_command_and_receive(filename="output", 
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
        starttime = time.time()
        # the following line updates the perceived information
        self.WingLoop_LogFile = read_aswing_file("output", 
                                                    self.WingLoop_LogFile, 
                                                    self.rename_map,
                                                    RecordStateHistory=True,
                                                compiled_pattern=self._compiled_pattern)
        
        # plot things now, if needed, or update the plot
        if self.WingLoop_IsPlotting and self.LivePlot:
            self.WingLoop_Liveplot.update(self.WingLoop_LogFile)
        idx = self.WingLoop_LogFile["_state_count"] - 1
        x_state = self.WingLoop_LogFile["ModelStates"][idx]
        
        self.writingtime += (time.time()-starttime)
        if not self.fixed_stick:
            output = self.PyControl.PyControl_DoControllerStep(instantaneous_state = x_state, Dt = Dt_aswing*K_aswing)
            print("[WingLoop] step = ", self.count,"; i+1 control:",output)

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

    def Time_Transient_Simulation(self,Dt,N,K,stop_if_notconverged = True):
        
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

            breaksim = any(not v for v in self.WingLoop_LogFile["ModelVariables"]["IsConverged"]["values"])
            if not breaksim:
                # performing a number K of iteration (write output of the previous state, obtain the control command, send to aswing, perform K aswing iterations)
                if self.print_output:
                    print("[WingLoop] iterations: from", self.count, " to ",self.count+K, " (step of ", K,")")
                self.Performing_K_iterations_ASWING(Dt_aswing=Dt, K_aswing=K)
                #incrementing the counter
                self.count += K
            else:
                break

        if not breaksim:
            ### PERFORMING THE FINAL ROUND OF ITERATIONS (between L+k*K and N)
            if self.count<N:
                if self.print_output:
                    print("Final iterations: from", self.count, " to ",N, " (step of ", N - self.count,")")
                self.Performing_K_iterations_ASWING(Dt_aswing=Dt, K_aswing=N-self.count)
                self.count += N-self.count
            if self.print_output:
                print("[WingLoop] Final Counter",self.count)
            breaksim = any(not v for v in self.WingLoop_LogFile["ModelVariables"]["IsConverged"]["values"])

        if not breaksim:
            allconverged = True
        else:
            print("[WingLoop] Unconverged Simulation, Good Luck")
            allconverged = False
        return allconverged

            
    def Outputting_The_State_File(self,statefile_filename=None):
        stdout, stderr = self.ASW_handler.send_command_and_receive("\n")
        stdout, stderr = self.ASW_handler.send_command_and_receive("HSAV")
        if statefile_filename == None:
            statefile_filename = self.WingLoop_LogFile["ModelName"]+".state"
        else:
            statefile_filename = statefile_filename+".state"
        stdout, stderr = self.ASW_handler.send_command_and_receive(statefile_filename,custom_timer=1)
        print("[WingLoop] Saved → "+statefile_filename)

### OUTPUT THE RESULTS

    def Outputting_The_Results(self, custom_filename = None):
        """_summary_

        Args:
            plot (bool, optional): do you want to show plots or not. Defaults to True.
            timeseries (string, optional): name of the timeseries to be saved. Defaults to None.
            
        Maybe one could create this with the function that stops only when the file is done writing
        """
        #if plot:
        #    self.PyControl.plot_the_data()
        ## writing an output timeseries for various variables
        #if timeseries is not None:
        #    print("TEST")
        #    stdout, stderr = self.ASW_handler.send_command_and_receive("P")
        #    stdout, stderr = self.ASW_handler.send_command_and_receive("W")
        #    if os.path.exists(timeseries+".t"):
        #        os.remove(timeseries+".t")
        #    stdout, stderr = self.ASW_handler.send_command_and_receive(timeseries+".t",custom_timer=1)
        print("[WingLoop] writingtime",self.writingtime)

        self.WingLoop_Liveplot.update(self.WingLoop_LogFile) #this will be the first plot if self.LivePlot = False
        print("[WingLoop] Simulation Ended, Press Enter to continue")
        #input()

        ## Exporting the data 
        if custom_filename == None:
            custom_filename = self.WingLoop_LogFile["ModelName"]
        export_data_dict(self.WingLoop_LogFile,custom_filename + ".json")

        if self.WingLoop_IsPlotting:
            self.WingLoop_Liveplot.export(custom_filename+".pdf")

    def Closing_WingLoop(self,removefiles = True):
        """Is in charge of closing the ASWING and ending the program

        Args:
            removefiles (bool, optional): If true, the input and output files for control are deleted. Defaults to True.
        """

        ### FINISHIING THE PROGRAM
        print("[WingLoop] Ending WingLoop...")
        time.sleep(0.5)
        # removing the communication files
        if removefiles:
            os.remove("output")
            os.remove("input")
        if self.WingLoop_IsPlotting:
            self.WingLoop_Liveplot.close()

        stdout, stderr = self.ASW_handler.send_command_and_receive("\n")  # go back to main menu
        # Quit Aswing
        time.sleep(0.5)
        self.ASW_handler.quit_and_close_aswing()
        print("[WingLoop] Closed WingLoop correctly!")


