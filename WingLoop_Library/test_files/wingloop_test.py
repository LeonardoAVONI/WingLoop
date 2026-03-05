import numpy as np
import time
import os
from WingLoop_Library.WingLoop_Library import WingLoop

print(os.listdir())

"""  
questo funziona, adesso bisogna metterci i parametri del controllo


"""

WL_Instance = WingLoop()
WL_Instance.Launch_ASWING(aswing_path ="/home/daep/l.avoni/Bureau/02_ASWING/ASWING_Stable/bin", # /home/leonardo-avoni/Desktop/02_ASWING/ASWING_Stable/bin
                          sim_directory = "test_aircraft",
                          asw_filename = "wingloop_test_aircraft.asw",
                          print_output = False,
                          timer_text=0.0001,
                          finished_writing_check_timestep = 0.01)

WL_Instance.Deactivate_Graphics()
WL_Instance.Load_Files("01_1minus_cosine.gust")
WL_Instance.Load_Files("wingloop_test_aircraft.pnt")
WL_Instance.Load_Files("wingloop_test_aircraft.set")
WL_Instance.Load_Files("wingloop_test_aircraft.state")

print("\tSTART")
Dt=0.01
N=10
K=1
start_time = time.time()
WL_Instance.Time_Transient_Simulation(Dt,N,K)
time_taken = time.time()-start_time
print("TIME TAKEN: ",time_taken)
WL_Instance.Outputting_The_Results(plot = False, timeseries="LQR")

#WL_Instance.Outputting_The_State_File("default_output.state")

