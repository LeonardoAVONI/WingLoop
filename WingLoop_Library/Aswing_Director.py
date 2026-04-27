# =============================================================================
# WingLoop — Aswing Director
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
Aswing Director

Author: Leonardo AVONI
Date: 21/10/2024
Email: avonileonardo@gmail.com
Last modified: 10/03/2026
Version 8

====================================================================================

Description:
    This Python class provides an interface to control the Fortran-based ASWING program 
    through the terminal using Python's subprocess module. It allows you to start the ASWING 
    process, send commands, read the program's output, and close the program cleanly. The 
    class handles non-blocking reads from ASWING's standard output and error streams by 
    using threading and queues for real-time interaction.
    
    Note that this class is made to send commands to ASWING; but there is no real guarantee 
    for the moment that ASWING is on time with Python. For example, we may be sending 7 commands,
    but in reality ASWING will still be working at command 1, and once it has finished he will be
    going for command 2. For instance: "send_command_and_receive" will just send an ASWING command 
    and that's it.
    
    The exception to that is when using "send_writefile_command_and_receive". This function will 
    wait until the filename is written by ASWING before continuing
    
    If we want to keep a somewhat real-time sync from Python to ASWING, we should use "wait_time=0.03"
    
How To Install
    Position yourself in the folder upstream of ASWING_Director_Library 
    (the folder containing this __init__.py code)
    Alongside that folder there should be the pyproject.toml
    Open a terminal, and write "/usr/bin/python3 -m pip install --user -e ."
    This will install it in Python, in editable mode (if you modify the code it 
    will impact the package behavior)
    Also: for VSCode to identify the imports: Add in the settings.json 
        (File>Preferences>Settings, then top right corner)file the following:
        "python.analysis.extraPaths": ["~/Bureau/01 Github/01_ASWING_Director"]

Features:
- Aswing_Director: Class instance to which we specify the ASWING version, wait_time and 
    finished_writing_file_check_timestep (the latter for "send_writefile_command_and_receive")
- start_aswing: we start ASWING, eventually with an input file to use, and also specifying 
    if we want prints or not. This print setting will be valid for all the code operation
- send_command_and_receive method: used to send commands to ASWING, as long as there is no
    need for time-synchronization
- send_writefile_command_and_receive: used to send commands to ASWING (usually X command for 
    transient simulation), without continuing until the state file we expect is ready from ASWING
- quit_and_close_aswing: does what it says method 

Note:
As for the Aswing program, use with multiple screens will lead to crash of the code. 
Hence, use this code in singlescreen only, or disable the graphics

Tutorial:
- an example is provided at the very end of this code

Changelog:
- Version 4 (21/10/2024): first decent version working
- Version 5 (28/10/2024):
    in function start_aswing: added the "directory" input
- Version 6 (30/10/2024): added "send_writefile_command_and_receive" function
    to be used for instructions requiring to write a file, while also waiting 
    for the file to be finished writing
- Version 7 (06/11/2024): added customizable check_timestep in "send_writefile_command_and_receive"
- Version 8 (03/12/2024): minor comments made
- Version 9 (17/12/2024): added file length check (>0) in "send_writefile_command_and_receive"
- Package Version (29/05/2025): the class is made package. Versions are now handled only via GitHub
- Version 8 (10/03/2026): this version is now included within the WingLoop package
====================================================================================
"""


import subprocess
import os
import time
import sys
import threading
from queue import Queue, Empty
import inotify_simple

class Aswing_Director:
    """
    A class to manage interaction with the ASWING software, enabling real-time communication, 
    command execution, and file handling for aerodynamic and flight analysis.
    Attributes:
        aswing_path (str): Path to the ASWING executable.
        wait_time (float): Default wait time used by send_command_and_receive for processing commands.
        finished_writing_file_check_timestep (float): Timestep used by send_writefile_command_and_receive for checking file write completion.
    Methods:
        start_aswing(directory=".", filename=" ", print_output=True):
            Starts the ASWING process, captures its output and error streams, and optionally prints them.
        send_command_and_receive(command, custom_timer=None):
            Sends a command to ASWING and retrieves its response, there's no time-sync with Python
        send_writefile_command_and_receive(filename, custom_timer=None, append_or_overwrite=None):
            Sends a command to write a file, waits for the file to be written, and retrieves the response. 
            Does not proceed until the file is ready
        quit_and_close_aswing():
            Sends the QUIT command to ASWING and closes the process.
    """
    
    def __init__(self, aswing_path,# "/home/daep/l.avoni/Bureau/02_ASWING/ASWING_Stable/bin/"
                 aswing_alias,
                 wait_time=0.03, #
                 finished_writing_file_check_timestep=0.001):
        
        if aswing_path is None:
            try:
                type_output = subprocess.check_output(
                    f"bash -ic 'type {aswing_alias}'",
                    shell=True,
                    text=True,
                    stderr=subprocess.DEVNULL
                ).strip()
                # Output is: "aswing_stable is aliased to `/path/to/aswing'"
                # Extract the path from between backtick and single quote
                import re
                match = re.search(r'`([^\']+)\'', type_output)
                if not match:
                    raise RuntimeError(f"Could not parse path from: {type_output}")
                aswing_exec = match.group(1)

            except subprocess.CalledProcessError:
                raise RuntimeError(
                    f"Could not find ASWING executable using alias '{aswing_alias}'. "
                    "Check your .bashrc alias."
                )

            if not aswing_exec:
                raise RuntimeError(
                    f"Alias '{aswing_alias}' did not resolve to an executable."
                )
            aswing_path = os.path.dirname(aswing_exec)

        self.aswing_path = aswing_path
        self.asw_process = None
        self.stdout_queue = Queue() #where outputs are written
        self.stderr_queue = Queue() #where error messages are written
        self.wait_time=wait_time
        self.finished_writing_file_check_timestep = finished_writing_file_check_timestep

    def enqueue(self, pipe, queue):
        """
        Read output from the process and put it in the queue
        """
        try:
            for line in iter(pipe.readline, ''):
                if line:
                    queue.put(line)
        finally:
            pipe.close()

    def start_aswing(self, directory=".", filename="hawk.asw",print_output=True):
        """
        This function starts ASWING, according to the previously defined ASWING 
        version from the Aswing_Director class declaration.
        Args:
            filename (str): a filename of the .asw geometry file to be used (hawk.asw for example)
            directory (str): The local folder where the geometry file is stored (. by default)
            print_output (bool): True or False depending on if we want the output to be printed or not during code execution
        """
        # saving the current working directory
        self.previous_directory = os.getcwd()
        self.print_output = print_output

        # going to the directory where the UAV files are
        os.chdir(directory)

        # starts ASWING, with the UAV file if provided
        start_command = [os.path.join(self.aswing_path, "aswing")]

        if filename:
            clean_name = filename.removesuffix(".asw")
            start_command.append(clean_name)
            print("[Aswing_Director]", clean_name)


        self.asw_process = subprocess.Popen(start_command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1)  # Line-buffered for real-time interaction          

        threading.Thread(target=self.enqueue, args=(self.asw_process.stdout, self.stdout_queue), daemon=True).start()
        threading.Thread(target=self.enqueue, args=(self.asw_process.stderr, self.stderr_queue), daemon=True).start()

        start_time = time.time() 
        """
        there is the need to make it run a few times in a loop because at the beginning it may be empty, idk
        if we take out this while, it will print the data but at the next command, idk
        """
        while time.time() - start_time < self.wait_time:
            try:
                if self.print_output:
                    print(self.stdout_queue.get_nowait(), end="")
            except Empty:
                pass

            try:
                if self.print_output:
                    print(self.stderr_queue.get_nowait(), end="")
            except Empty:
                pass
        
        # changing the directory to the none we had previously
        #os.chdir(previous_directory)


    def send_command_and_receive(self, command, custom_timer=None):
        """
        Function used to send a command (OPER for example) to ASWING. The function returns
        the printed output of ASWING to that command; but depending on how wait_time was defined, 
        the reported output may not be in real time.
        If we wish to wait longer for the code to send the output, we can define a custom_timer 
        (in seconds) different from wait_time defined during Aswing_Director class initialization
        
        Args:
            command (str): command to send (like OPER)
            custom_timer (float): optional wait_time, if we do not want to use the default one from class initialization
        """
        standard_output = []
        error_output = []

        if custom_timer is not None:
            timer=custom_timer
        else:
            timer= self.wait_time

        start_time = time.time()

        # Send command
        self.asw_process.stdin.write(command + "\n") #send the command with \n
        self.asw_process.stdin.flush() #also sends the command, in case \n does not work
        # Collect output within timeout period
        while time.time() - start_time < timer:
            try:
                stdout_line = self.stdout_queue.get_nowait()
                standard_output.append(stdout_line)
                if self.print_output:
                    print(stdout_line, end="")
            except Empty:
                pass

            try:
                stderr_line = self.stderr_queue.get_nowait()
                error_output.append(stderr_line)
                if self.print_output:
                    print(stderr_line, end="")
            except Empty:
                pass
        return ''.join(standard_output), ''.join(error_output)

    def send_writefile_command_and_receive_old(self, filename, custom_timer=None, append_or_overwrite = None):
        """
        Function used when iterating the X commands of WingLoop
        The function asks to write the file, and waits until the file is written to continue, granting some type of 
        synchronization between Python and ASWING. It returns the outputs of the "send_command_and_receive" function, as well as 
        the time taken for the function to do its job; keep in mind that here the time measured is not the real time, since ASWING
        runs in parallel to python most of the time
        
        Args:
            filename (str): filename we expect to be written
            custom_timer (float, optional): Used to wait after the command is sent. If None, the default wait_time is used.
            append_or_overwrite (str, optional): defines if we want to append "a" the file, or overwrite "o" it.

        """
 
        internal_start_time = time.time()
        tt=custom_timer
        a,b = self.send_command_and_receive(filename, custom_timer=tt)
        
        # provide the possibility to append or overwrite the output file, instead of deleting it each time
        if append_or_overwrite:
            c,d = self.send_command_and_receive(append_or_overwrite, custom_timer=tt)            
            # storing the output here, for the output
            a = a+c
            b = b+d       
        
        timeout=5
        last_size = -1
        
        # for the following: decoupling the "is the file there" part, with the "how long is it" helps speed up slightly the time
        # also, when checking the file length, there are actually 2 terations: first the file is length 0, then final length; no in-between apparently

        while time.time() - internal_start_time < timeout:
            if os.path.exists(filename):
                #buffer_time_internal_internal=time.time()
                break
            time.sleep(0.00000001)  # Check every 10 mus

        condition=1
        while time.time() - internal_start_time < timeout:
            if os.path.exists(filename):
                #current_size = os.path.getsize(file_path)
                current_size = os.stat(filename).st_size
                if current_size == last_size:
                    #checking if the file has actually something written inside
                    if current_size>0:
                        condition=0
                        break
                last_size = current_size
            time.sleep(self.finished_writing_file_check_timestep)
            #print(check_timestep)
        if condition:
            raise NameError("Filename takes too much time to write (ASWING)!")

        return ''.join(a), ''.join(b), time.time()-internal_start_time

    def send_writefile_command_and_receive(self, filename, custom_timer=None, append_or_overwrite=None):
        internal_start_time = time.time()
        tt = custom_timer
        a, b = self.send_command_and_receive(filename, custom_timer=tt)

        if append_or_overwrite:
            c, d = self.send_command_and_receive(append_or_overwrite, custom_timer=tt)
            a, b = a + c, b + d

        timeout = 99999  # seconds

        # ── inotify path: wake up exactly when ASWING closes the file ──
        watch_dir  = os.path.dirname(os.path.abspath(filename))
        watch_name = os.path.basename(filename)

        inotify = inotify_simple.INotify()
        wd = inotify.add_watch(watch_dir, inotify_simple.flags.CLOSE_WRITE)
        try:
            deadline = time.time() + timeout
            while time.time() < deadline:
                ms_left = max(1, int((deadline - time.time()) * 1000))
                for event in inotify.read(timeout=ms_left):
                    if event.name == watch_name:
                        if os.stat(filename).st_size > 0:
                            return ''.join(a), ''.join(b), time.time() - internal_start_time
        finally:
            inotify.rm_watch(wd)
            inotify.close()

    def Load_Files_ASW(self,filename):
        """
        From the main ASWING menu, this function loads the file called "filename", 
        according to its extension. When using ASWING from the terminal, any file 
        extension can be loaded. Here the extension will make the difference

        Args:
            filename (string): filename to enter in ASWING from the main menu, including the extension
        """

        if filename.endswith(".asw"):
            stdout, stderr = self.send_command_and_receive("load "+filename) #asw
        elif filename.endswith(".pnt"):
            stdout, stderr = self.send_command_and_receive("pget "+filename) # pnt
        elif filename.endswith(".set"):
            stdout, stderr = self.send_command_and_receive("sget "+filename) # set
        elif filename.endswith(".state"):
            stdout, stderr = self.send_command_and_receive("tget "+filename) # state
        elif filename.endswith(".gust"):
            stdout, stderr = self.send_command_and_receive("gget "+filename) # gust
        else:
            print("UNKNOWN FILENAME")
            sys.exit()


    def quit_and_close_aswing(self):
        """
        Send QUIT command to ASWING and closes the process.
        """
        self.send_command_and_receive("QUIT")
        time.sleep(self.wait_time)
        self.asw_process.stdin.close()
        self.asw_process.stdout.close()
        self.asw_process.stderr.close()
        self.asw_process.wait()
        if self.print_output:
            print("ASWING is closed")



if __name__=="__main__":
    ### TUTORIAL ###
    ASW_handler = Aswing_Director(aswing_path = None,aswing_alias = "aswing_stable")
    print("A")
    ASW_handler.start_aswing(filename="test_files/test_aircraft/wingloop_test_aircraft",print_output=True)
    print("B")
    # Send commands and print outputs
    stdout, stderr = ASW_handler.send_command_and_receive("plot")
    print(stdout)
    print("C")
    stdout, stderr = ASW_handler.send_command_and_receive("P")
    print("D")
    stdout, stderr = ASW_handler.send_command_and_receive("4")
    print("E")
    stdout, stderr = ASW_handler.send_command_and_receive("H")
    print("F")
    stdout, stderr = ASW_handler.send_command_and_receive("D")
    print("G")
    stdout, stderr = ASW_handler.send_command_and_receive("H")
    print("H")
    stdout, stderr = ASW_handler.send_command_and_receive("\n")
    stdout, stderr = ASW_handler.send_command_and_receive("OPER")
    stdout, stderr = ASW_handler.send_command_and_receive("!V 30")
    stdout, stderr = ASW_handler.send_command_and_receive("%")
    stdout, stderr = ASW_handler.send_command_and_receive("A")
    stdout, stderr = ASW_handler.send_command_and_receive("D")
    stdout, stderr = ASW_handler.send_command_and_receive("H")
    stdout, stderr = ASW_handler.send_command_and_receive("x")
    stdout, stderr = ASW_handler.send_command_and_receive("w")
    stdout, stderr = ASW_handler.send_command_and_receive("test")
#    stdout, stderr = ASW_handler.send_writefile_command_and_receive(filename = "test",append_or_overwrite="A")
    stdout, stderr = ASW_handler.send_command_and_receive("\n")

    print("I")
    ASW_handler.quit_and_close_aswing()

