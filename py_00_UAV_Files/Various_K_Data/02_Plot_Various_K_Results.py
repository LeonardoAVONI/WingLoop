import numpy as np
import matplotlib.pyplot as plt

# Load all data
data = {}
for i in [1, 2, 3, 5]:  # From K_1 to K_5
    name = f"K_{i}_"
    data[name] = {
        "TIME": np.load(name + "TIME.npy"),
        "PITCH": np.load(name + "PITCH.npy"),
        "ALPHA": np.load(name + "ALPHA.npy"),
        "EarthZ": np.load(name + "EarthZ.npy"),
        "VELOCITY": np.load(name + "VELOCITY.npy"),
    }

# Create the figure with 4 subplots
fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
fig.subplots_adjust(hspace=0.4, top=0.9)  # Adjust top for suptitle

# Define plotting order and titles
keys = ["PITCH", "EarthZ", "ALPHA", "VELOCITY"]
titles = ["Pitch [°]", "EarthZ [m]", "Alpha [°]", "Velocity [m/s]"]

y_limits_min = [-2, -1, -4, 29.5]
y_limits_max = [2, 2, 1.5, 30.5]

# Plot each vector type in its subplot
for idx, (key, title, ym, ymax) in enumerate(zip(keys, titles, y_limits_min, y_limits_max)):
    ax = axs[idx]
    for name in data:
        k_value = name.split("_")[1]  # Extract the number after "K_"
        ax.plot(data[name]["TIME"], data[name][key], label=f"K = {k_value} [#]")  # TIME as x-axis
    ax.set_ylabel(title)
    if idx == 1:
        ax.legend(loc='upper left')
    ax.grid(True)
    ax.set_ylim([ym, ymax])

# Finalize
axs[-1].set_xlabel("Time [s]")
fig.suptitle(
    "00_Hawk_V0, trimmed 30m/s, 1-cosine (5 -15 0 15 3 0) Kp 7.75, Ti 0.275, Td 0.0625\n"
    "No anti windup, no fixed initial aileron deflection, K variable, Ts = 0.02 [s], no ground effect, ailerons at ±10, default settings",
    fontsize=10
)
plt.show()
