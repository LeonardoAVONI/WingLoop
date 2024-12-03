"""
====================================================================================
View Geometry, Version 1

Author: Leonardo AVONI
Date: 06/11/2024
Email: avonileonardo@gmail.com

Last modified: 06/11/2024

====================================================================================

Description:
This Python code is made to be used with Aswing_Director library. Its goal is to view the used geometry in the current folder

Features:
    For the PLOT test 
    - Start ASWING hawk
    - Deactivate the graphics (PLPA, to use with multiple screens) and the terminal scripts
    - Plot and save 2 plots from the PLOT menu

====================================================================================
"""
import sys

# Importing Paths for the project
sys.path.append('../py_02_Utilities')

import subprocess
import os
import sys
import time
import threading
from queue import Queue, Empty
from  Aswing_Director_Library import Aswing_Director


# Get the directory of the current script file
script_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
os.chdir(script_dir)

print_output_or_not=1

ASW_handler = Aswing_Director()
ASW_handler.start_aswing(filename="00_Hawk_V0",print_output=print_output_or_not)

# Deactivate Graaphics
stdout, stderr = ASW_handler.send_command_and_receive("PLPA", print_output=print_output_or_not)
stdout, stderr = ASW_handler.send_command_and_receive("G", print_output=print_output_or_not)
stdout, stderr = ASW_handler.send_command_and_receive("\n", print_output=print_output_or_not)

# Create plots in PLOT menu, and make 2 hardcopies (undeflected)
stdout, stderr = ASW_handler.send_command_and_receive("plot", print_output=print_output_or_not)
stdout, stderr = ASW_handler.send_command_and_receive("D", print_output=print_output_or_not)
stdout, stderr = ASW_handler.send_command_and_receive("H", print_output=print_output_or_not)
stdout, stderr = ASW_handler.send_command_and_receive("\n", print_output=print_output_or_not)

# Quit Aswing
ASW_handler.quit_and_close_aswing(print_output=print_output_or_not)

