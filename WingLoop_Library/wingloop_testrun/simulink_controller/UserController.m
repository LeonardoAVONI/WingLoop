%{
====================================================================================
WingLoop Library — UserController (MATLAB)

Author: Leonardo AVONI
Date: 08/11/2024
Email: avonileonardo@gmail.com

Last modified: 12/03/2026

====================================================================================

Description:
    MATLAB handle class implementing the UserController interface expected by
    PyControl for the 'matlab' and 'simulink' backends.

    On construction the class either loads pre-computed workspace data from a
    .mat file, or runs computeInitialData() to produce default values.  The
    three workspace properties (workspace_scalar, workspace_string,
    workspace_matrix) are read by PyControl after construction and pushed into
    the MATLAB base workspace so that Simulink models can access them at sim()
    time.

    The step() method is called once per simulation timestep by PyControl and
    must return a struct whose fields are the controller output signals
    (F1-F20, E1-E20 as required by the application).

    PID controller
    --------------
    Because MATLAB does not allow two classdef blocks in one file, the PID
    state is stored as scalar properties of UserController and the update
    logic lives in the private runPID() method.  This is exactly equivalent
    to instantiating a separate PIDController class.

====================================================================================
%}

classdef UserController < handle

    % ──────────────────────────────────────────────────────────────────────
    % Properties
    % ──────────────────────────────────────────────────────────────────────
    properties
        % Required by PyControl — do not remove
        simulationtime      % Accumulated simulation time (s)
        workspace_scalar    % Scalar double shared with the MATLAB base workspace
        workspace_string    % String shared with the MATLAB base workspace
        workspace_matrix    % Matrix shared with the MATLAB base workspace

        % Controller reference values (set in computeInitialData)
        F2ref               % Trim flap 2 deflection [deg]
        E2ref               % Trim engine 2 setting
        Pitchref            % Pitch angle reference [rad]

        % PID gains
        Kp                  % Proportional gain
        Ki                  % Integral gain
        Kd                  % Derivative gain

        % PID internal state — equivalent to the Python PIDController instance
        pid_previous_error
        pid_previous_I_output
        pid_previous_D_output
    end

    % ──────────────────────────────────────────────────────────────────────
    % Public methods
    % ──────────────────────────────────────────────────────────────────────
    methods

        % ------------------------------------------------------------------
        function obj = UserController(precomputed_file_path)
            %{
            UserController  Constructor — initialise workspace data and PID.

            If precomputed_file_path points to an existing .mat file the
            workspace properties are loaded from it via loadFromPrecomputed().
            Otherwise computeInitialData() is called to produce defaults.
            simulationtime and all PID state variables are always reset to
            zero regardless of the data source.

            Parameters
            ----------
            precomputed_file_path : string (optional, default "")
                Path to a .mat file.  Pass an empty string to use defaults.
            %}

            arguments
                precomputed_file_path string {mustBeText} = ""
            end

            obj.workspace_scalar = [];
            obj.workspace_string = "";
            obj.workspace_matrix = [];

            fprintf('[MATLAB UserController] path? %s', precomputed_file_path);

            if ~isempty(precomputed_file_path) && isfile(precomputed_file_path)
                obj.loadFromPrecomputed(precomputed_file_path);
            else
                obj.computeInitialData();
            end

            obj.simulationtime = 0.0;

            % ── PID gains (mirror of Python: Kp=100, Ki=100, Kd=20) ──────
            obj.Kp = 100.0;
            obj.Ki = 100.0;
            obj.Kd = 20.0;

            % ── PID state — all zeroed at construction ────────────────────
            obj.pid_previous_error    = 0.0;
            obj.pid_previous_I_output = 0.0;
            obj.pid_previous_D_output = 0.0;

            fprintf('[MATLAB UserController] Initialized\n');
            fprintf('   scalar  = %.4f\n',      obj.workspace_scalar);
            fprintf('   string  = %s\n',         obj.workspace_string);
            fprintf('   matrix size = %d x %d\n', size(obj.workspace_matrix));
            fprintf('   matrix =\n');
            disp(obj.workspace_matrix);
            fprintf('   PID gains: Kp=%.1f  Ki=%.1f  Kd=%.1f\n', obj.Kp, obj.Ki, obj.Kd);

            assignin('base', 'Kp', obj.Kp);
            assignin('base', 'Ki', obj.Ki);
            assignin('base', 'Kd', obj.Kd);


            obj.F2ref    = -4.638;
            obj.E2ref    =  2.7460;
            obj.Pitchref =  0.081889689;   % [rad]
            assignin('base','F2ref',    obj.F2ref);
            assignin('base','E2ref',    obj.E2ref);
            assignin('base','Pitchref', obj.Pitchref);

        end

        % ------------------------------------------------------------------
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

        % ------------------------------------------------------------------
        function computeInitialData(obj)
            fprintf('[MATLAB UserController] Computing default initial data...\n');
            obj.workspace_scalar = 1234.5678;
            obj.workspace_string = "test_matlab_workspace";
            obj.workspace_matrix = obj.workspace_scalar * eye(5);

            % Trim / reference values — mirror of Python _compute_from_startup
            obj.F2ref    = -4.638;
            obj.E2ref    =  2.7460;
            obj.Pitchref =  0.081889689;   % [rad]
        end


    end % public methods


end % classdef