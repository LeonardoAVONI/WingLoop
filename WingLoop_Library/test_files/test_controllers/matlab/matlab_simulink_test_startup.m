% startup.m

workspace_scalar = 1234.5678;
workspace_string = "test_matlab_workspace";
workspace_matrix = workspace_scalar * eye(5);

disp("[matlab/simulink startup] workspace_scalar =")
disp(workspace_scalar)
disp("[matlab/simulink startup] workspace_string =")
disp(workspace_string)
disp("[matlab/simulink startup] workspace_matrix =")
disp(workspace_matrix)