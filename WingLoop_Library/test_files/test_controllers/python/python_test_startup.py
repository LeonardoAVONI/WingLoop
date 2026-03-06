# startup.py
import numpy as np

def compute_initial_data(x_state_trimmed):
    """
    Example: dummy but dimensionally consistent computation
    In reality you would run your LQR / eigen / whatever here
    """
    n_states = len(x_state_trimmed)          # e.g. 1945
    n_modes  = 32                            # example reduced modal size
    n_inputs = 6                             # example (4 flaps + 2 engines)

    # Fake but correctly shaped matrices
    W_T_M = np.random.randn(n_modes, n_states) * 0.01
    W_T_M[0, :10] = 1.0                      # just to make it non-zero somewhere

    K_x = np.random.randn(n_inputs, n_states) * 0.005
    K_x[:, -6:] = np.eye(6) * (-0.8)         # strong gain on inputs for trimming

    q_trim = W_T_M @ x_state_trimmed

    return {
        'K_x': K_x,
        'W_T_M': W_T_M,
        'q_state_trimmed': q_trim
    }
    
if __name__=="__main__":
# create_precomputed.py (run once manually)
    x_trim = np.zeros(1945)          # dummy
    data = compute_initial_data(x_trim)     # from startup.py
    np.savez("precomputed_python.npz", **data)