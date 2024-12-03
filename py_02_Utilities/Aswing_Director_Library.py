"""
====================================================================================
Aswing Director, Version 8

Author: Leonardo AVONI
Date: 21/10/2024
Email: avonileonardo@gmail.com

Last modified: 03/12/2024

====================================================================================

Description:
    This Python class provides an interface to control the Fortran-based ASWING program 
    through the terminal using Python's subprocess module. It allows you to start the ASWING 
    process, send commands, read the program's output, and close the program cleanly. The 
    class handles non-blocking reads from ASWING's standard output and error streams by 
    using threading and queues for real-time interaction.
    
    Note that this class is made to send commands to ASWING; but there is no real guarantee 
    for the moment that ASWING is on time with Python. For example, we may be sending 7 commands,
    but in reality ASWING will still be working at command 1

Features:
- Create a Director handler with Aswing_Director
- Start Aswing with start_aswing (a filename can be loaded)
- Commands can be sent via the send_command_and_receive method
- Aswing is closed with the quit_and_close_aswing method 

Comments:
- A timer ("wait_time") is added during class initialization to ensure appropriate 
  delays for commands like plotting.
- Custom timeout values can be passed to the send_command_and_receive function for 
  operations that are known to take longer time to complete
- printing scripts can be deactivated by setting for each function print_output=0

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
====================================================================================
"""


import subprocess
import os
import time
import threading
from queue import Queue, Empty


class Aswing_Director:
    def __init__(self, aswing_path="/home/daep/l.avoni/Documents/Aswing_R_5_98/bin/",wait_time=0.030):
        self.aswing_path = aswing_path
        self.asw_process = None
        self.stdout_queue = Queue() #where outputs are written
        self.stderr_queue = Queue() #where error messages are written
        self.wait_time=wait_time


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

    def start_aswing(self, directory=".", filename=" ",print_output=1):
        """
        Start ASWING and launch threads to capture output and error streams.
        """
        # saving the current working directory
        previous_directory = os.getcwd()

        # going to the directory where the UAV files are
        os.chdir(directory)

        start_command = [os.path.join(self.aswing_path, "aswing")]
        if filename.strip():
            start_command.append(filename)

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
                if print_output:
                    print(self.stdout_queue.get_nowait(), end="")
            except Empty:
                pass

            try:
                if print_output:
                    print(self.stderr_queue.get_nowait(), end="")
            except Empty:
                pass
        
        # changing the directory to the none we had previously
        #os.chdir(previous_directory)


    def send_command_and_receive(self, command, print_output=1, custom_timer=None):
        """
        Sends a command and listens for a response. Timeout ensures we don't wait forever.
        """
        standard_output = []
        error_output = []


        if custom_timer:
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
                if print_output:
                    print(stdout_line, end="")
            except Empty:
                pass

            try:
                stderr_line = self.stderr_queue.get_nowait()
                error_output.append(stderr_line)
                if print_output:
                    print(stderr_line, end="")
            except Empty:
                pass
        return ''.join(standard_output), ''.join(error_output)

    def send_writefile_command_and_receive(self, filename, print_output=1, custom_timer=None, check_timestep=0.001):
        """
        returns the outputs of the "send_command_and_receive" function, as well as the time taken for the function to do its job;
        keep in mind that here the time measured is not the real time, since aswing runs in parallel to python most of the time
        
        """
        
        internal_start_time = time.time()
        tt=custom_timer
        pp=print_output
        a,b = self.send_command_and_receive(filename, print_output=pp, custom_timer=tt)
        # test
        #time.sleep()
        
        
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
                #print(current_size)
                if current_size == last_size:
                    condition=0
                    break
                last_size = current_size
            time.sleep(check_timestep)  # Check every 1 ms
            #print(check_timestep)
        if condition:
            raise NameError("Filename takes too much time to write (ASWING)!")

        # at this point, aswing is synchronous with python, assuming the write part took plac correctly. We call this place TOTEM


        return ''.join(a), ''.join(b), time.time()-internal_start_time

    def quit_and_close_aswing(self,print_output=1):
        """
        Send QUIT command to ASWING and close the process.
        """
        buffer=print_output
        self.send_command_and_receive("QUIT", print_output=buffer)
        time.sleep(self.wait_time)
        self.asw_process.stdin.close()
        self.asw_process.stdout.close()
        self.asw_process.stderr.close()
        self.asw_process.wait()
        if print_output:
            print("ASWING is closed")


#print(os. getcwd())


if __name__=="__main__":
    ### TUTORIAL ###
    ASW_handler = Aswing_Director()
    print("A")
    ASW_handler.start_aswing(filename="hawk",print_output=1)
    print("B")
    # Send commands and print outputs
    stdout, stderr = ASW_handler.send_command_and_receive("plot", print_output=1)
    print("C")
    stdout, stderr = ASW_handler.send_command_and_receive("P", print_output=1)
    print("D")
    stdout, stderr = ASW_handler.send_command_and_receive("4", print_output=1)
    print("E")
    stdout, stderr = ASW_handler.send_command_and_receive("H", print_output=1)
    print("F")
    stdout, stderr = ASW_handler.send_command_and_receive("D", print_output=1)
    print("G")
    stdout, stderr = ASW_handler.send_command_and_receive("H", print_output=1)
    print("H")
    stdout, stderr = ASW_handler.send_command_and_receive("\n", print_output=1)
    print("I")
    ASW_handler.quit_and_close_aswing(print_output=1)

