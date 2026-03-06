classdef UserController < handle
    % UserController - MATLAB user-defined controller class
    % Mirrors Python UserController: handles own initialization

    properties
        workspace_scalar    % double
        workspace_string    % string
        workspace_matrix    % matrix
    end

    methods
        function obj = UserController(precomputed_file_path)
            % Constructor - only needs precomputed path (or empty)
            arguments
                precomputed_file_path string {mustBeText} = ""
            end

            obj.workspace_scalar = [];
            obj.workspace_string = "";
            obj.workspace_matrix = [];

            if ~isempty(precomputed_file_path) && isfile(precomputed_file_path)
                obj.loadFromPrecomputed(precomputed_file_path);
            else
                obj.computeInitialData();
            end

            % Minimal validation / debug print
            fprintf('[MATLAB UserController] Initialized\n');
            fprintf('   scalar  = %.4f\n', obj.workspace_scalar);
            fprintf('   string  = %s\n',   obj.workspace_string);

            fprintf('   matrix size = %d x %d\n', size(obj.workspace_matrix));
            fprintf('   matrix =\n');
            disp(obj.workspace_matrix);
        end


        function loadFromPrecomputed(obj, mat_path)
            fprintf('[MATLAB UserController] Loading: %s\n', mat_path);
            data = load(mat_path);

            if isfield(data, 'workspace_scalar')
                obj.workspace_scalar = double(data.workspace_scalar);
            end
            if isfield(data, 'workspace_string')
                obj.workspace_string = string(data.workspace_string);
            end
            if isfield(data, 'workspace_matrix')
                obj.workspace_matrix = double(data.workspace_matrix);
            end
        end


        function computeInitialData(obj)
            % Default / dummy computation when no precomputed file
            fprintf('[MATLAB UserController] Computing default initial data...\n');

            obj.workspace_scalar = 1234.5678;
            obj.workspace_string = "test_matlab_workspace";
            obj.workspace_matrix = obj.workspace_scalar * eye(5);
        end


        function output = step(obj, instantaneous_state, Dt)
            % Controller step logic
            % instantaneous_state is a column vector (double array)

            fprintf('[MATLAB step] Using:\n');
            fprintf('   scalar  = %.4f\n', obj.workspace_scalar);
            fprintf('   string  = %s\n',   obj.workspace_string);

            fprintf('   matrix size = %d x %d\n', size(obj.workspace_matrix));
            fprintf('   matrix =\n');
            disp(obj.workspace_matrix);

            % Example: very simple passthrough + tiny bias from property
            testvalue = instantaneous_state(1:6);

            output = struct(...
                'F1', testvalue(1), ...
                'F2', testvalue(2), ...
                'F3', testvalue(3), ...
                'F4', testvalue(4), ...
                'E1', testvalue(5), ...
                'E2', testvalue(6) ...
            );
        end
    end
end