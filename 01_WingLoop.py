"""
====================================================================================
Controlled Flight Text, Version 2

Author: Leonardo AVONI
Date: 25/10/2024
Email: avonileonardo@gmail.com

Last modified: 17/12/2024
quick test
====================================================================================

Description:
This Python code is made to be used with Aswing_Director library. 
Its goal is to perform a time-transient simulation on a flexible hawk aircraft
The code runs as follows
    First the script settings and simulation parameters are specified (Ts, N, K, L, Dt...)
    The PID parameters are specified
    Then graphics are deactivated
    Then the initial state is imported
    THe simulation settings are imported
    A 1-cosine gust profile is imported
    Ground effect is disabled
    Altitude units are set to km
    Also, the output text file is made lighter by not-showing beam-wise variables
    Then a first L iterations are computed
    Then the other iterations are performed by groups of K iterations at a time, with control (either via matlab or via python)
    Then the results can be plotted and/or the score can be displayed
    The time results are also written to a csv file for easy access



Features:
-control in matlab and python
-Aswing_V2 use (modified to go beyond 201 op.points limit)
-time measurements (to check which part of the code take how much time)
-added the possibility to use angular velocity from ASWING, and increased read/write precision using ASWING V3
-added ASWING output file checks, avoiding crashes

Next improvements:
-Simulink Bridge
-independence on the aircraft used
-better organization of the control part, making a better class

====================================================================================
"""

import csv
import subprocess
import numpy as np
import os
import time
import matlab.engine #executing this line steals a bit of time
import threading
from queue import Queue, Empty
from py_02_Utilities.Aswing_Director_Library import Aswing_Director
from py_02_Utilities.text2python_library import text2python_main, python2text, text2python_withderivative
from py_02_Utilities.Control_Library import Control
import sys

times = {}
global internal_times
internal_times = {}
internal_times["counter"]=0
internal_times["write_data_from_aswing_time"]=0
internal_times["process_data_from_aswing_time"]=0
internal_times["compute_PID_instructions_time"]=0
internal_times["write_flap_commands_time"]=0
internal_times["compute_K_iterations_time"]=0
internal_times["full_iteration_time"] = 0

start_time = time.time()

def py_01_Performing_K_iterations_ASWING(count, Aswing_handler, Matlab_handler, Python_handler, Dt_aswing, K_aswing, print_setting, ttimer, ttimer_check):
    #global last_modification_time
    global internal_times

    """
    Works from the Oper menu, made to append K_aswing iterations with control to current simulation
    The function does the following:
        writes current state from ASWING to a .txt file
        obtains from such state the (instantaneous) variables used for control
        appends the instantaneous variables to the previous time series of previously appended values
            this operation can be done in Matlab or Python, but here it's done in Python
        asking the correspondent control function for the controls needed, according to a control scheme defined in Control_Library
        writing a text file to be used by ASWING for control (defining elevator inputs, engine input...)
        send K_aswing iterations in ASWING with the generated control text file

    """
    internal_times["counter"] +=1
    start_time_internal = time.time()
    if count: #check if we are doing the very first time-transient simulation
        command=str(Dt_aswing)+" -"+str(K_aswing)
    else:
        command=str(Dt_aswing)+" "+str(K_aswing)
    x_command="x input"
    
    """
    Several options are available concerning the state file written by ASWING
        delete: this option deletes the older "output" file, so the new one has no problems being written
        overwrite: this method overwrites current "output" file
        none: this method does not write any file. It can be used for time testing
    
    note that time.sleep() are needed lower in the code if "none" is used
    """
    
    state_file_write_options = "delete"
    

    
    if state_file_write_options != "none": 
        if state_file_write_options == "delete":
            stdout, stderr = Aswing_handler.send_command_and_receive("W", print_output=print_setting,custom_timer=ttimer) # write the data
            os.remove("output")
            stdout, stderr , time_taken= Aswing_handler.send_writefile_command_and_receive(filename="output", print_output=print_setting, custom_timer=ttimer,check_timestep=ttimer_check) # , check_timestep=0.05

        if state_file_write_options == "overwrite": 
            stdout, stderr = Aswing_handler.send_command_and_receive("W", print_output=print_setting,custom_timer=ttimer) # write the data
            if True: #deleting the content method
                with open('output', 'w') as file: #deletes the previously written data from the output file, it now will have a length 0
                    pass
                while os.stat("output").st_size: #checkig that the file is still there, but his content has been deleted
                    time.sleep(0.0000001)
            else: #modification time based
                print("THE OVERWRITE MODIFICATION, THAT RELIES ON CHECKING THE output FILE STATE (check if it was modified or not) ENDED UP NOT WORKING")
                sys.exit()
                #while os.stat("output").st_mtime == last_modification_time:
                #    time.sleep(0.00001)
                #last_modification_time = os.stat("output").st_mtime
            stdout, stderr , time_taken= Aswing_handler.send_writefile_command_and_receive(filename="output", print_output=print_setting, custom_timer=ttimer,check_timestep=ttimer_check , append_or_overwrite="O") # , check_timestep=0.05

    internal_times["write_data_from_aswing_time"] += time.time()-start_time_internal
    buffer_time_internal=time.time()
    
    # writing a dictionary containing the flight data specifics
    
    #instantaneous_flight_data = text2python_main("output")
    instantaneous_flight_data = text2python_withderivative("output")    
    print(instantaneous_flight_data)
    # writing flight data to python workspace database, not updated
    
    if Matlab_handler is None:
        # Appending the data in the python script
        Python_handler.append_flight_data(instantaneous_flight_data)

        internal_times["process_data_from_aswing_time"] += time.time()-buffer_time_internal # time to process the data from aswing in python
        buffer_time_internal=time.time()

        # Controlling through the python script
        output = Python_handler.UAV_control_Strategy(instantaneous_flight_data, Dt = Dt_aswing*K_aswing)

        internal_times["compute_PID_instructions_time"] += time.time()-buffer_time_internal # time to compute the flap PID instructions in python
        buffer_time_internal=time.time()
    else:
        # appending in matlab
        Matlab_handler.append_flight_data(instantaneous_flight_data,nargout=0)

        # obtain the needed controls through Matlab (matlab already has states up-to-date)
        output = Matlab_handler.control_law(instantaneous_flight_data,Dt_aswing)

    #write matlab output to input text file
    python2text("input",output)

    internal_times["write_flap_commands_time"] += time.time()-buffer_time_internal # time to write the flap instructions for aswing
    buffer_time_internal=time.time()

    #perform K time iteration on aswing, using the input text file just written
    stdout, stderr = Aswing_handler.send_command_and_receive(x_command, print_output=print_setting,custom_timer=ttimer) #create K_aswing iteration of the unsteady simulation
    stdout, stderr = Aswing_handler.send_command_and_receive(command , print_output=print_setting,custom_timer=ttimer)

    internal_times["compute_K_iterations_time"] += time.time()-buffer_time_internal # time to compute K iterations

    internal_times["full_iteration_time"] +=time.time()-start_time_internal
    print(internal_times)

# Additional Matlab Directory
matlab_script_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "py_02_Utilities")

# Get the directory of the current script file
script_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
os.chdir(script_dir)

#in case we want to show script output
print_output_or_not=1

#in case we want to use matlab (1) or python (0) for control
use_matlab=0

# Simulation Specifics
N = 300 # amount of time steps in ASWING, INTEGER larger than 1
Dt = 0.01 # ASWING Time step [s]
K = 10 # sendig the files to Matlab every K-th ASWING iteration, INTEGER
L = 1 #beginning iterations

# timer used by default for the Aswing_Director functions
timer_text=0.0000001 #

# timer used to check for the last update on the size of the state file written by ASWING
check_timestep=0.001 #seconds

# what file
filename_string="00_Hawk_V0"

# what control? (in the case of the rigid aircraft)
# here the PID parameters are Kp, Ti and Td. when entering the controller, they are converted to Kp, Ki = Kp/Ti and Kd = Kp*Td


if filename_string=="00_Hawk_V0":
    p_k=7.75   # [-]
    p_i=0.275  # [s]
    p_d=0.0625 # [s]

if K !=1: #for K=10
    p_k=2  # [-]
    p_i=0.5  # [s]
    p_d=0.1 # [s]

ASW_handler = Aswing_Director(aswing_path="/home/daep/l.avoni/Bureau/02_ASWING/Aswing_V3/bin/")
ASW_handler.start_aswing(directory="py_00_UAV_Files",filename=filename_string,print_output=print_output_or_not)

# Deactivate Graphics
stdout, stderr = ASW_handler.send_command_and_receive("PLPA", print_output=print_output_or_not)
stdout, stderr = ASW_handler.send_command_and_receive("G", print_output=print_output_or_not)
stdout, stderr = ASW_handler.send_command_and_receive("\n", print_output=print_output_or_not)

# Perform Simulations in Oper
"""
load the set of constraints needed for trimmed flight computation
instead of building them one by one in the OPER>T menu

We will here load a .state file including
    -the free flight constraints
    -the fixed-stick trimmed aircraft constrol surface deflections and engine power
The .state file does not contain the simulation specifics i.e. steady or unsteady

The trimmed .state file, for the moment, must be created manually
"""
# Setting up the Beginning of the Resolution: the trimmed state
stdout, stderr = ASW_handler.send_command_and_receive("tget "+filename_string+"_leveled_30.state", print_output=print_output_or_not) # load the constraints fand trimmed state
# Importing the settings needed for the OPER resolution
stdout, stderr = ASW_handler.send_command_and_receive("sget 01_fast_settings.set", print_output=print_output_or_not) # load the settings to be used in OPER
# Importing the gust needed
stdout, stderr = ASW_handler.send_command_and_receive("gget 01_1minus_cosine.gust", print_output=print_output_or_not) # load the gust (1-cosine)
# Set altitude units (from ground) to km instead of kilofeet
stdout, stderr = ASW_handler.send_command_and_receive("UALT", print_output=print_output_or_not) #go to oper menu
# Go to OPER, time-transient simulation menu
stdout, stderr = ASW_handler.send_command_and_receive("oper", print_output=print_output_or_not) #go to oper menu
# Set the parameters for Ground Effect
stdout, stderr = ASW_handler.send_command_and_receive("G", print_output=print_output_or_not) #Ground Effect Parameters
stdout, stderr = ASW_handler.send_command_and_receive("0", print_output=print_output_or_not) #No Ground Effect
stdout, stderr = ASW_handler.send_command_and_receive("0 0 1", print_output=print_output_or_not) #Ground normal vector (not really useful since no ground effect)
# Set the altitude from ground
stdout, stderr = ASW_handler.send_command_and_receive("A", print_output=print_output_or_not) #Altitude from ground, in km (unit previously set)
stdout, stderr = ASW_handler.send_command_and_receive("0", print_output=print_output_or_not) #Altitude from ground
stdout, stderr = ASW_handler.send_command_and_receive("\t", print_output=print_output_or_not)
# Set the simulation to time-transient
stdout, stderr = ASW_handler.send_command_and_receive(".", print_output=print_output_or_not) #switch to unsteady simulation


# Lines for reduced text
if True:
    stdout, stderr = ASW_handler.send_command_and_receive("K", print_output=print_output_or_not) #go in the OPER/settings menu
    stdout, stderr = ASW_handler.send_command_and_receive("V", print_output=print_output_or_not) #what are the variables listed (either = or W) spanwise each beam
    stdout, stderr = ASW_handler.send_command_and_receive("2 3 5 7 8 12 22 27", print_output=print_output_or_not) #deselect all
    stdout, stderr = ASW_handler.send_command_and_receive("\n", print_output=print_output_or_not)

stdout, stderr = ASW_handler.send_command_and_receive("x", print_output=print_output_or_not)


### PERFORMING JUST THE FIRST ITERATION (starting from the pre-computed )
#create a dummy output file
if not os.path.exists("output"):
    open("output", "w")
# the following comments concern a version based with overwrite, that is working badly
#with open('output', 'w') as file: 
#    pass
#global last_modification_time 
#last_modification_time = -1

if use_matlab:
    #starting Matlab
    print("Starting Matlab")
    matlab_engine = matlab.engine.start_matlab()
    # Add the directory to MATLAB's path
    matlab_engine.addpath(matlab_script_dir)
    python_control= None
else:
    print("Using Python for Control")
    python_control = Control(K=p_k,Ti=p_i,Td=p_d)
    matlab_engine = None

times["starting_time"] = time.time()-start_time
buffertime = time.time()

# command for the first L time steps
if L: #if L is non-zero, then L iterations are done with the aircraft in steady state, and flaps and engine fixed at steady-state positions
    command=str(Dt)+" "+str(L)
    stdout, stderr = ASW_handler.send_command_and_receive("x", print_output=print_output_or_not,custom_timer=timer_text) #very first time-transient iteration
    stdout, stderr = ASW_handler.send_command_and_receive(command , print_output=print_output_or_not,custom_timer=timer_text)
counter=L

times["first_iterations_time"] = time.time()-buffertime
buffertime = time.time()

# timer to be used in case state_file_write_options = none
#timer_text=0.018 # for K=1
#timer_text=0.19 # for K=10

### PERFORMING ALL INTERMEDIATE ITERATIONS (between L and N)
while not (counter + K >= N):
    # performing a number K of iteration (write output of the previous state, obtain the control command, send to aswing, perform K aswing iterations)
    if print_output_or_not:
        print("Internal iterations: from", counter, " to ",counter+K, " (step of ", K,")")
    py_01_Performing_K_iterations_ASWING(count=counter,Aswing_handler=ASW_handler, Matlab_handler=matlab_engine, Python_handler=python_control, Dt_aswing=Dt, K_aswing=K, print_setting=print_output_or_not, ttimer=timer_text, ttimer_check = check_timestep)
    #incrementing the counter
    counter = counter+K

times["middle_iterations_time"] = time.time()-buffertime
buffertime = time.time()

# performing the final round of iterations
if counter<N:
    if print_output_or_not:
        print("Final iterations: from", counter, " to ",N, " (step of ", N - counter,")")
    py_01_Performing_K_iterations_ASWING(count=counter, Aswing_handler=ASW_handler,  Matlab_handler=matlab_engine , Python_handler=python_control, Dt_aswing=Dt, K_aswing=N-counter, print_setting=print_output_or_not, ttimer=timer_text, ttimer_check = check_timestep)
    counter += N-counter
if print_output_or_not:
    print("Final Counter",counter)
times["final_iterations_time"] = time.time()-buffertime
buffertime = time.time()

### OUTPUT THE RESULTS

if not use_matlab:
    python_control.plot_the_data()
    #score_new = python_control.obtain_the_metrics()
    #print("SCORE: ",score_new)
    pass


### FINISHIING THE PROGRAM
print("Finishing the Program")
time.sleep(0.5)
# removing the communication files
if True:
    os.remove("output")
    os.remove("input")
# closing the matlab engine
if use_matlab:
    matlab_engine.quit() 

# writing an output timeseries for various variables
if False:
    stdout, stderr = ASW_handler.send_command_and_receive("P" , print_output=print_output_or_not)
    stdout, stderr = ASW_handler.send_command_and_receive("S" , print_output=print_output_or_not)
    stdout, stderr = ASW_handler.send_command_and_receive("11" , print_output=print_output_or_not)
    stdout, stderr = ASW_handler.send_command_and_receive("20" , print_output=print_output_or_not)
    stdout, stderr = ASW_handler.send_command_and_receive("17" , print_output=print_output_or_not)
    stdout, stderr = ASW_handler.send_command_and_receive("66" , print_output=print_output_or_not)
    stdout, stderr = ASW_handler.send_command_and_receive("55" , print_output=print_output_or_not)
    stdout, stderr = ASW_handler.send_command_and_receive("68" , print_output=print_output_or_not)
    stdout, stderr = ASW_handler.send_command_and_receive("\t" , print_output=print_output_or_not)
    stdout, stderr = ASW_handler.send_command_and_receive("\t" , print_output=print_output_or_not)
    stdout, stderr = ASW_handler.send_command_and_receive("W" , print_output=print_output_or_not)
    stdout, stderr = ASW_handler.send_command_and_receive("timeseries_WingLoop" , print_output=print_output_or_not)

# write a backup
#stdout, stderr = ASW_handler.send_command_and_receive("\n" , print_output=print_output_or_not)  # go back to main menu
#stdout, stderr = ASW_handler.send_command_and_receive("hsav backup.state" , print_output=print_output_or_not)  # go back to main menu

stdout, stderr = ASW_handler.send_command_and_receive("\n" , print_output=print_output_or_not)  # go back to main menu
# Quit Aswing
ASW_handler.quit_and_close_aswing(print_output=print_output_or_not)

times["closing_time"] = time.time()-buffertime
times["total_time"] = time.time()-start_time

print(times)
print(internal_times)

#writing the python data file to something python accessible
python_control.write_to_file()

#write the data of the time taken by the code to a csv file
if False:
    with open('output_data.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        
        # Writing headers for the first dictionary
        writer.writerow(['Metric', 'Value'])
        for key, value in times.items():
            writer.writerow([key, value])

        for key, value in internal_times.items():
            writer.writerow([key, value])