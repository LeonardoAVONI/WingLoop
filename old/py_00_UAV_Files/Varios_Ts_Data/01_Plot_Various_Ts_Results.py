"""
====================================================================================
Plot Various Ts Results

The following folder contains the .bin data for various numpy vectors describing Pitch(Time) for various scenarios
Here, the following Python PID has been used:
	Kp = 7.75  [-]
	Ti = 0.275 [s]
	Td = 0.0625 [s]
	No "anti-windup"
	No fixed initial aileron deflection
	K was set to 1
	No ground effect
	Aileron deflection limited to pm10 [°]
Ts was made variable in order to assess its impact on the control specifics
A 1-cosine with the following parameters has been used
	 5	! 1-cosine vertical gust
	-15	! horizontal distance over x until beginning of gust (starts after 0.5s)
	 0 	! coefficient for y slope
	 15	! Horizontal gust length H (full rise at 1s from simulation beginning)
	 3	! Wmax
	 0
The controlled system was the 00_Hawk_V0.asw, a 4m-wingspan rigid aircraft. All aileron deflections were set to the leveled flight trimming configuration, except the elevator, i.e. F2, that was made to control the pitch:
	Pitch = 0.5706 [°]
	Engine = 7.559 [N] (trimmed for 30m/s)
At the beginning of the simulation, the PID must "build" its integral term, thus leading to a small jump in the aileron deflection, later stabilizing at the trimming point position i.e. 1.479 [°]

The following configurations were tested:
	Conv_6 Ts 0.005 N 300
	Conv_5 Ts 0.01  N 150
	Conv_4 Ts 0.02  N 75
	Conv_3 Ts 0.03  N 50
	Conv_2 Ts 0.05  N 30
	Conv_1 Ts 0.1   N 15
	
We decided to plot the following quantities
	pitch [°]
	velocity [m/s]
	aoa [°]
	z position [m]
We noticed the folllowing quantities to stay constant (reasonable because longitudinal dynamics only)
	sideslip
	y position (Earth)
	yaw
	roll
Concerning x position (Earth), we decided that since the velocity and the pitch were already available, and roll and yaw were zero, ant there were no horizontal wind, then it would be redundant

====================================================================================

I need to plot the data for Conv 1 to 6
    Pitch
    Alpha
    EarthZ
    Velocity

"""

import numpy as np
import matplotlib.pyplot as plt

# Load all data
data = {}
for i in range(1, 7):  # From Conv_1 to Conv_6
    name = f"Conv_{i}"
    data[name] = {
        "TIME": np.load(name + "TIME.npy"),
        "PITCH": np.load(name + "PITCH.npy"),
        "ALPHA": np.load(name + "ALPHA.npy"),
        "EarthZ": np.load(name + "EarthZ.npy"),
        "VELOCITY": np.load(name + "VELOCITY.npy"),
    }

# Create the figure with 4 subplots
fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
fig.subplots_adjust(hspace=0.4)  # Space between subplots

# Define plotting order and titles
keys = ["PITCH", "EarthZ", "ALPHA", "VELOCITY"]
titles = ["Pitch [°]", "EarthZ [m]", "Alpha [°]", "Velocity [m/s]"]
Ts = [1E-1, 5E-2, 3E-2, 2E-2, 1E-2, 5E-3]

y_limits_min = [-2,-1,-4,29.5]
y_limits_max = [2,2,1.5,30.5]

# Plot each vector type in its subplot
for idx, (key, title, ym, ymax) in enumerate(zip(keys, titles, y_limits_min, y_limits_max)):
    ax = axs[idx]
    for name in data:
        number = int(name.replace("Conv_", ""))-1
        ts = np.format_float_scientific(Ts[number], precision=1, unique=False, exp_digits=1)
        ax.plot(data[name]["TIME"], data[name][key], label="Ts = "+str(ts)+" [s]")  # TIME as x-axis
    #ax.set_title(title)
    ax.set_ylabel(title)
    if idx ==1:
        ax.legend(loc='upper left')
    ax.grid(True)
    ax.set_ylim([ym, ymax])

# Finalize
axs[-1].set_xlabel("Time [s]")
fig.suptitle("00_Hawk_V0, trimmed 30m/s, 1-cosine (5 -15 0 15 3 0) Kp 7.75, Ti 0.275, Td 0.0625\n No anti windup, no fixed initial aileron deflection, K set to 1, no ground effect, ailerons at pm10, default settings")
plt.show()
