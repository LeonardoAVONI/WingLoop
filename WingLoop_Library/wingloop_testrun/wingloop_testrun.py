import numpy as np
import time
import os
from WingLoop_Library import WingLoop
WL_Instance = WingLoop()

print(os.listdir())



Dt=0.01
N=100
K=1

WL_Instance = WingLoop()

""" 

simulink_test_controller.fmu
simulink_test_controller.slx
python_test_controller.py


"""

WL_Instance.Launch_WingLoop_Control(cntrl_directory = "matlab_controller", 
                          cntrl_filename = "UserController.m",
                          timestep = Dt,
                          precomputed_filename = None, 
                          rebuild_fmu_file = False,
                          show_simulink_window = False)
WL_Instance.InitializePlot(liveplot = True,
                          plot_variables= ["earth X", "earth Y", "earth Z", "Heading", "Elev.", "Bank"],
                          plot_sim_time = N*Dt,
                          plot_refreshtime= 0.00001,
                          plot_size = (16, 10))

WL_Instance.Launch_ASWING(aswing_fullpath =None, # /home/leonardo-avoni/Desktop/02_ASWING/ASWING_Stable/bin
                          aswing_alias = "aswing_stable", #aswing_stable , aswing_5_98
                          sim_directory = "aswing_geometry",
                          asw_filename = "wingloop_test_aircraft.asw",
                          print_output = False,
                          timer_text=0.0001,
                          finished_writing_check_timestep = 0.01)

WL_Instance.Deactivate_Graphics()
WL_Instance.Load_Files("01_1minus_cosine.gust")
WL_Instance.Load_Files("wingloop_test_aircraft.pnt")
WL_Instance.Load_Files("wingloop_test_aircraft.set")
WL_Instance.Load_Files("wingloop_test_aircraft.state")


tstart = time.time()
WL_Instance.Time_Transient_Simulation(Dt,N,K)
tend = time.time()
print("totaltime",tend-tstart)



WL_Instance.Outputting_The_State_File(statefile_filename="ciao")

WL_Instance.Outputting_The_Results(custom_filename = "ciao")

WL_Instance.Closing_WingLoop(removefiles = True)

