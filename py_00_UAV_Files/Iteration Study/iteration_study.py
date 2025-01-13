import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def import_the_data(name):
    

    # Read the file while skipping comment lines
    df = pd.read_csv(name, delimiter=r"\s+", comment='#', skip_blank_lines=True)

    # Assign column names manually since the file doesn't have headers
    df.columns = ["i", "t", "Wy_rot_rate", "Theta_elev", "Flap_2", "Int_Th_Thc_dt", "V", "alpha"]

    # Convert columns to NumPy arrays
    i = df["i"].to_numpy()
    t = df["t"].to_numpy()
    Wy_rot_rate = df["Wy_rot_rate"].to_numpy()
    Theta_elev = df["Theta_elev"].to_numpy()
    Flap_2 = df["Flap_2"].to_numpy()
    Int_Th_Thc_dt = df["Int_Th_Thc_dt"].to_numpy()
    V = df["V"].to_numpy()
    alpha = df["alpha"].to_numpy()
    
    if False:
        # Print to verify
        print("i:", i)
        print("t:", t)
        print("Wy_rot_rate:", Wy_rot_rate)
        print("Theta_elev:", Theta_elev)
        print("Flap_2:", Flap_2)
        print("Int_Th_Thc_dt:", Int_Th_Thc_dt)
        print("V:", V)
        print("alpha:", alpha)
    
    return t, Wy_rot_rate, Theta_elev, Int_Th_Thc_dt, Flap_2


test = "PID"

Theta_ref=0.5706
F2_ref=1.47927

if test == "P":
    P=4
    I=0
    D=0
elif test == "PI":
    P=2
    I=1
    D=0
elif test == "PD":
    P=7.75
    I=0
    D=7.75
elif test == "PID":
    P=7.75
    I=28.1818181818
    D=0.484375

def PID_estimation(theta,theta_dot,Int_theta,P,I,D,ref_theta,ref_F2):
    estimated_F2 = ref_F2 -(ref_theta-theta)*P-D*(0-theta_dot)-I*(0-Int_theta)
    return estimated_F2

Time, Theta_dot, Theta, Int_Theta, Flap = import_the_data(test+"_output.t")

Flap_Estimated = PID_estimation(Theta,Theta_dot,Int_Theta,P,I,D,Theta_ref,F2_ref)

# Plot Flap and Flap_Estimated over Time
plt.figure(figsize=(8, 5))
plt.plot(Time, Flap, label="Flap", linestyle='-', color='b')
plt.plot(Time, Flap_Estimated, label="Flap Estimated", linestyle='--', color='r')

# Labels and title
plt.xlabel("Time (s)")
plt.ylabel("Flap Value")
plt.title("Flap vs Flap Estimated Over Time")
plt.legend()
plt.grid(True)

# Show the plot
plt.show()