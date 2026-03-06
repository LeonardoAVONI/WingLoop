import numpy as np

workspace_scalar_py = 12340.5678
workspace_string_py = "test_python_workspace_database"
workspace_matrix_py = workspace_scalar_py * np.eye(5)

np.savez(
    "precomputed_python.npz",
    workspace_scalar_py=workspace_scalar_py,
    workspace_string_py=workspace_string_py,
    workspace_matrix_py=workspace_matrix_py
)