# startup.py
import numpy as np

def compute_initial_data():
    """
    Example: dummy but dimensionally consistent computation
    In reality you would run your LQR / eigen / whatever here
    """


    workspace_scalar_py = 1234.5678
    workspace_string_py = "test_python_workspace_database"
    workspace_matrix_py = workspace_scalar_py * np.eye(5)


    return {
        'workspace_scalar_py': workspace_scalar_py,
        'workspace_string_py': workspace_string_py,
        'workspace_matrix_py': workspace_matrix_py
    }
    