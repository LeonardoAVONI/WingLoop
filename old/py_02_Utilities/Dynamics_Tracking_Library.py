"""
====================================================================================
Dynamics_Tracking_Library, Version 2

Author: Leonardo AVONI
Date: 06/11/2024
Email: avonileonardo@gmail.com

Last modified: 08/11/2024

====================================================================================

Description:
This Python class provides a place to store flight variables that are incremended at each Kth timestep

Features:
-This class was not used, since the same thing is available in Control_Library

Comments:
-

Changelog:
- Version 1 (06/11/2024): first version of the code
- Version 2 (08/11/2024): added the 10 variables from TIME to VELOCITY


====================================================================================
"""
import numpy as np

class Dynamics_Tracking:
    def __init__(self):
        # Initialize as None to check whether they've been assigned
        self.TIME = None

        self.EarthX = None
        self.EarthY = None
        self.EarthZ = None

        self.PITCH = None
        self.ROLL = None
        self.YAW = None

        self.ALPHA = None
        self.BETA = None
        self.VELOCITY = None

    def append_flight_data(self, instantaneous_struct):
        # Check if the arrays are initialized

        if self.Time is None:
            # If they aren't, initialize them
            self.TIME = np.array([instantaneous_struct["Time"]])

            self.PITCH = np.array([instantaneous_struct["Pitch"]])
            self.ROLL = np.array([instantaneous_struct["Bank"]])
            self.YAW = np.array([instantaneous_struct["Heading"]])

            self.EarthX = np.array([instantaneous_struct["earthX"]])
            self.EarthY = np.array([instantaneous_struct["earthY"]])
            self.EarthZ = np.array([instantaneous_struct["earthZ"]])

            self.ALPHA = np.array([instantaneous_struct["Alpha"]])
            self.BETA = np.array([instantaneous_struct["Beta"]])
            self.VELOCITY = np.array([instantaneous_struct["Velocity"]])

        else:
            # If they are initialized, append the new values
            self.TIME = np.append(self.TIME, instantaneous_struct["Time"])

            self.PITCH = np.append(self.PITCH, instantaneous_struct["Pitch"])
            self.ROLL = np.append(self.ROLL, instantaneous_struct["Bank"])
            self.YAW = np.append(self.YAW, instantaneous_struct["Heading"])

            self.EarthX = np.append(self.EarthX, instantaneous_struct["earthX"])
            self.EarthY = np.append(self.EarthY, instantaneous_struct["earthY"])
            self.EarthZ = np.append(self.EarthZ, instantaneous_struct["earthZ"])

            self.ALPHA = np.append(self.ALPHA, instantaneous_struct["Alpha"])
            self.BETA = np.append(self.BETA, instantaneous_struct["Beta"])
            self.VELOCITY = np.append(self.VELOCITY, instantaneous_struct["Velocity"])

