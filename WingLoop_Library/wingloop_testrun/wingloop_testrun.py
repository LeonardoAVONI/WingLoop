# =============================================================================
# WingLoop — wingloop_testrun.py
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


import numpy as np
import time
import os
from WingLoop_Library import WingLoop


def main():
    """ 
    wingloop_testrun.py

    Author: Leonardo AVONI
    Date: 25/10/2024
    Email: avonileonardo@gmail.com
    Version 2.0

    Last modified: 10/03/2026

    This is a test case allowing you to run WingLoop and check if it is working correctly
    It will:
        1) Declare some WingLoop simulation parameters
        2) Open a WingLoop Instance
        3) Initialize the control method (defined in PyControl.py)
        4) Initialize the WingLoop plot method (defined in PyControl_Plot.py)
        5) Launching ASWING with the required geometry and parameters
        6) Importing files, and deactivating ASWING graphs
        7) Performing the WingLoop time-transient simulation
        8) Save the WingLoop and ASWING data
        9) Close ASWING and WingLoop

    For more details, refer to the header of each library used

    """

    #     1) Declare some WingLoop simulation parameters
    Dt=0.01 # timestep [s]
    N=300 # total iterations to perform
    K=1     # each K iterations we send the ASWING state to PyControl. Leave to 1 if you do not want to add lag

    #     2) Open a WingLoop Instance
    WL_Instance = WingLoop()


    #     3) Initialize the control method (defined in PyControl.py)
    """ 
    Available control folders:
        matlab_controller
        simulink_controller
        python_controller
    Available controllers:
        simulink_test_controller.fmu (need to compile it first)
        simulink_test_controller.slx
        python_test_controller.py
        UserController.m
    """
    
    selector = "sim"
    if selector == "sim":
        print("[wingloop_testrun] controller = Simulink")
        control_dir = "simulink_controller"
        control_filename = "simulink_test_controller.slx"
    elif selector == "mat":
        print("[wingloop_testrun] controller = Matlab")
        control_dir = "matlab_controller"
        control_filename = "UserController.m"
    elif selector == "fmu":
        print("[wingloop_testrun] controller = Simulink FMU")
        control_dir = "simulink_controller"
        control_filename = "simulink_test_controller.fmu"
    elif selector == "py":
        print("[wingloop_testrun] controller = Python")
        control_dir = "python_controller"
        control_filename = "python_test_controller.py"
        

    WL_Instance.Launch_WingLoop_Control(cntrl_directory = control_dir, 
                            cntrl_filename = control_filename,
                            timestep = Dt,
                            precomputed_filename = None, 
                            rebuild_fmu_file = True,
                            show_simulink_window = True)

    #     4) Initialize the WingLoop plot method (defined in PyControl_Plot.py)
    WL_Instance.InitializePlot(liveplot = True,
                            plot_variables= ["earth X", "earth Y", "earth Z", "Heading", "Elev.", "Bank"],
                            plot_sim_time = N*Dt,
                            plot_refreshtime= 1,
                            plot_size = (16, 10),
                            N_steps = N)

    #     5) Launching ASWING with the required geometry and parameters
    WL_Instance.Launch_ASWING(aswing_fullpath =None,                    # full aswing path
                            aswing_alias = "aswing_stable",           # if no path is provided, you can feed an alias
                            sim_directory = "aswing_geometry",        # geometry relative path (where aswing and wingloop files will be stored)
                            asw_filename = "wingloop_test_aircraft.asw", # geometry used
                            print_output = False,                     # note that the ASWING output is not printed "in real time"
                            timer_text=0.0001,                        # don't change 
                            finished_writing_check_timestep = 0.01)   # don't change 

    #     6) Importing files, and deactivating ASWING graphs
    WL_Instance.Deactivate_Graphics()
    WL_Instance.Load_Files("01_1minus_cosine.gust")         # gust file
    WL_Instance.Load_Files("wingloop_test_aircraft.pnt")    # operation point file
    WL_Instance.Load_Files("wingloop_test_aircraft.set")    # settings file
    WL_Instance.Load_Files("wingloop_test_aircraft.state")  # (precomputed) state file

    #     7) Performing the WingLoop time-transient simulation
    tstart = time.time()
    WL_Instance.Time_Transient_Simulation(Dt,N,K)
    tend = time.time()
    print("[wingloop_testrun] total simulation time",tend-tstart)


    #     8) Save the WingLoop and ASWING data
    WL_Instance.Outputting_The_State_File(statefile_filename=None) #one can specify custom names
    WL_Instance.Outputting_The_Results(custom_filename = None)     #one can specify custom names 

    #     9) Close ASWING and WingLoop
    WL_Instance.Closing_WingLoop(removefiles = False)

if __name__ == '__main__':
    main()