# startup.py  (used for both python and simulink_fmu)
import numpy as np

def compute_initial_data(x_state_trimmed):
    n_states = len(x_state_trimmed)
    n_modes  = 32
    n_inputs = 6

    W_T_M = np.random.randn(n_modes, n_states) * 0.01
    K_x   = np.random.randn(n_inputs, n_states) * 0.005
    q_trim = W_T_M @ x_state_trimmed

    return {
        'K_x': K_x.flatten(),           # FMU usually wants flat arrays
        'W_T_M': W_T_M.flatten(),
        'q_trim': q_trim.flatten(),
        # you can add scalars too: 'controller_Dt': 0.01, etc.
    }