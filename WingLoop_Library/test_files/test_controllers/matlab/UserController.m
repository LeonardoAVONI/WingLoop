%{
====================================================================================
WingLoop Library — UserController (MATLAB)

Author: Leonardo AVONI
Date: 08/11/2024
Email: avonileonardo@gmail.com

Last modified: 08/03/2026

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
    (F1–F20, E1–E20 as required by the application).


How the X vector from ASWING is structured:
==============================================================================
ASWING STATE VECTOR (X) ORGANIZATION (1-based indexing)
==============================================================================
1. DISTRIBUTED VARIABLES: i = 1 to (18 * IITOT)
   Organized in blocks of 18 for each spanwise station j:
   [1-3]   x, y, z (Location)           [10-12] Fx, Fy, Fz (Forces)
   [4-6]   phi, theta, psi (Angles)     [13-15] ux, uy, uz (Velocities)
   [7-9]   Mx, My, Mz (Moments)         [16-18] wx, wy, wz (Rotation Rates)
2. JOINTS: i = (18 * IITOT + 1) to (18 * IITOT + 12 * NJOIN)
   Organized in blocks of 12 for each joint:
   [1-3]   x, y, z (Position)           [7-9]   Mx, My, Mz (Joint Moments)
   [4-6]   ang1, ang2, ang3 (Angles)    [10-12] Fx, Fy, Fz (Joint Forces)
3. CIRCULATION (Gamma): i = (18 * IITOT + 12 * NJOIN + 1) to (+ NNTOT)
   Contains aerodynamic circulation quantities for each element.
4. GLOBAL VARIABLES (KPFREE): i = (Offset1 + 1) to (Offset1 + KPFREE)
   Offset1 = (18*IITOT + 12*NJOIN + NNTOT)
   [1-6]   Udotx-z, Wdotx-z             [16-18] Heading, Elev, Bank
   [7-12]  Ux-z, Wx-z                   [19-38] Flaps (F1 - F20)
   [13-15] Earth X, Y, Z                [39-50] Engines (E1 - E12)
   [51-62] Error Integrators (Vinf, Beta, Alpha, Phi, Theta, Psi, ROT, VAC)
5. ADDITIONAL PARAMETERS (KPTOT): i = (Offset1 + 63) to (Offset1 + KPTOT)
   [63-65] Aircraft CG (X, Y, Z)        [79-81] Aero Coeffs (Roll, Pitch, Yaw)
   [66-68] Vinf, Beta, Alpha            [82]    Span Efficiency
   [69-71] VIAS, Beta_ref, Alpha_ref    [83-88] Aero Force/Moment (Total)
   [72-74] Drag, Side, Lift (Forces)    [89-94] Accel. Reaction Force/Moment
   [75-78] Coeffs (CDi, CD, CY, CL)     [95-106] Ground & Engine Reaction F/M
   [107-112] Trefftz-Plane Aero F/M     [113-119] Mach, Alt, Rho, a, mu, g, Load
==============================================================================

====================================================================================
%}

classdef UserController < handle

    % ──────────────────────────────────────────────────────────────────────
    % Properties
    % ──────────────────────────────────────────────────────────────────────
    properties
        simulationtime  % Accumulated simulation time (s). DO NOT REMOVE — read by PyControl.
        workspace_scalar    % Scalar double shared with the MATLAB base workspace.
        workspace_string    % String shared with the MATLAB base workspace.
        workspace_matrix    % Matrix shared with the MATLAB base workspace.
    end

    % ──────────────────────────────────────────────────────────────────────
    % Methods
    % ──────────────────────────────────────────────────────────────────────
    methods

        % ------------------------------------------------------------------
        function obj = UserController(precomputed_file_path)
            %{
             UserController  Constructor — initialise workspace data.
            
             If precomputed_file_path points to an existing .mat file the
             workspace properties are loaded from it via loadFromPrecomputed().
             Otherwise computeInitialData() is called to produce defaults.
             simulationtime is always reset to 0.0 regardless of the source.
            
             Parameters
             ----------
             precomputed_file_path : string (optional, default "")
                 Path to a .mat file containing workspace_scalar,
                 workspace_string, and/or workspace_matrix fields.
                 Pass an empty string or omit to use computed defaults.  
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

            fprintf('[MATLAB UserController] Initialized\n');
            fprintf('   scalar  = %.4f\n', obj.workspace_scalar);
            fprintf('   string  = %s\n',   obj.workspace_string);
            fprintf('   matrix size = %d x %d\n', size(obj.workspace_matrix));
            fprintf('   matrix =\n');
            disp(obj.workspace_matrix);
        end

        % ------------------------------------------------------------------
        function loadFromPrecomputed(obj, mat_path)
            %{
             loadFromPrecomputed  Load workspace properties from a .mat file.
            
             Reads workspace_scalar, workspace_string, and workspace_matrix
             from the supplied .mat file.  Any field absent from the file is
             left at its default (empty / "").  Non-existent fields are
             silently skipped.
            
             Parameters
             ----------
             mat_path : char | string
                 Path to the .mat file to load.
            %}

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
            %{
            computeInitialData  Compute default workspace properties.
            
             Called when no valid precomputed file is supplied.  Sets
             workspace_scalar, workspace_string, and workspace_matrix to
             hard-coded test values.  Replace with real computation as needed.
            %}

            fprintf('[MATLAB UserController] Computing default initial data...\n');
            obj.workspace_scalar = 1234.5678;
            obj.workspace_string = "test_matlab_workspace";
            obj.workspace_matrix = obj.workspace_scalar * eye(5);
        end

        % ------------------------------------------------------------------
        function output = step(obj, instantaneous_state, Dt)
            %{
            step  Execute one controller timestep.
            
             Called once per simulation step by PyControl.  Increments
             simulationtime, applies control logic using the workspace
             properties and the current state, and returns a struct of
             output signals.  Add or remove output fields (F1–F20, E1–E20)
             to match the application; PyControl discovers them automatically.
            
             Parameters
             ----------
             instantaneous_state : double column vector
                 Current plant state, length N.
             Dt : double
                 Timestep duration in seconds.
            
             Returns
             -------
             output : struct
                 Struct whose fields are the controller output signals.
            %}

            obj.simulationtime = obj.simulationtime + Dt;

            fprintf('[MATLAB step] Using:\n');
            fprintf('   scalar  = %.4f\n', obj.workspace_scalar);
            fprintf('   string  = %s\n',   obj.workspace_string);
            fprintf('   matrix size = %d x %d\n', size(obj.workspace_matrix));
            fprintf('   matrix =\n');
            disp(obj.workspace_matrix);

            testvalue = instantaneous_state(1:6);

            output = struct( ...
                'F1',  1,...%testvalue(1) + obj.workspace_scalar, ...
                'F2',  2,...%testvalue(2) + obj.workspace_scalar, ...
                'F3',  3,...%testvalue(3), ...
                'F4',  4,...%testvalue(4), ...
                'E1',  5,...%testvalue(5), ...
                'E2',  6,...%testvalue(6), ...
                'E15', obj.simulationtime  ...
            );
        end

    end % methods
end % classdef