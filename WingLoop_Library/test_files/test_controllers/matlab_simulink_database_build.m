% startup.m, database build

%{
workspace_scalar
workspace_string
workspace_matrix
%}

workspace_scalar = 12340.5678;
workspace_string = "test_matlab_workspace_database";
workspace_matrix = workspace_scalar * eye(5);

% Save variables to MAT database
save('precomputed.mat', 'workspace_scalar', 'workspace_string', 'workspace_matrix');