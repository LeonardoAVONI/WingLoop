%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================

function run_wingloop_simulink_setup( ...
    T_sim, Dt_asw, Dt_sim, Ts_control, tau, t_int, ...
    ASW_FILE, PNT_FILE, SET_FILE, STATE_FILE, GUST_FILE, ...
    ROM, FullModel, u_trim, ...
    model_name, model_file ...
)

    fprintf('\n==================================================\n');
    fprintf('WingLoop Simulink Setup\n');
    fprintf('==================================================\n');

    %% --- Print Time Configuration ---
    fprintf('\n--- TIME CONFIGURATION ---\n');
    fprintf('T_sim      : %g s\n', T_sim);
    fprintf('Dt_ASWING  : %g s\n', Dt_asw);
    fprintf('Dt_Simulink: %g s\n', Dt_sim);
    fprintf('Ts_Control : %g s\n', Ts_control);
    fprintf('tau        : %g s\n', tau);
    fprintf('t_int      : %g s\n', t_int);

    %% --- Print ASWING Case Configuration ---
    fprintf('\n--- ASWING CASE CONFIGURATION ---\n');
    fprintf('ASW file  : %s\n', ASW_FILE);
    fprintf('PNT file  : %s\n', PNT_FILE);
    fprintf('SET file  : %s\n', SET_FILE);
    fprintf('STATE file: %s\n', STATE_FILE);
    fprintf('GUST file : %s\n', GUST_FILE);

    %% --- Print Model Dimensions ---
    NumModalStates    = ROM.n_modal;
    NumPhysicalStates = FullModel.n_orig + FullModel.n_in;

    fprintf('\n--- MODEL DIMENSIONS ---\n');
    fprintf('ROM.n_modal         : %d\n', ROM.n_modal);
    fprintf('FullModel.n_orig    : %d\n', FullModel.n_orig);
    fprintf('FullModel.n_in      : %d\n', FullModel.n_in);
    fprintf('NumModalStates      : %d\n', NumModalStates);
    fprintf('NumPhysicalStates   : %d\n', NumPhysicalStates);

    %% --- Print Trim Inputs ---
    fprintf('\n--- TRIM INPUTS ---\n');
    fprintf('u_trim = [');
    fprintf(' %.8g', u_trim);
    fprintf(' ]\n');

    %% --- Repository Paths ---
    % This file is expected to be located in:
    % WingLoop/WingLoop_Library/wingloop_testrun/simulink_controller/
    simulink_controller_path = fileparts(mfilename('fullpath'));
    wingloop_testrun_path = fileparts(simulink_controller_path);

    json_path = fullfile(wingloop_testrun_path, 'sim_config.json');

    addpath(simulink_controller_path);

    %% --- Write Python Configuration JSON ---
    config_data = struct( ...
        'T_sim', T_sim, ...
        'Dt_asw', Dt_asw, ...
        'ASW_FILE', ASW_FILE, ...
        'PNT_FILE', PNT_FILE, ...
        'SET_FILE', SET_FILE, ...
        'STATE_FILE', STATE_FILE, ...
        'GUST_FILE', GUST_FILE ...
    );

    fid = fopen(json_path, 'w');
    if fid < 0
        error('Unable to open sim_config.json for writing: %s', json_path);
    end

    fprintf(fid, '%s', jsonencode(config_data));
    fclose(fid);

    fprintf('\n[WL_main] Configuration written to:\n%s\n', json_path);

    %% --- Load Simulink Model ---
    load_system(model_file);

    %% --- Configure AswingPlant Block ---
    aswing_block_path = [model_name '/Non-Linear Plant (WingLoop)/MATLAB System1'];

    set_param(aswing_block_path, 'NumModalStates', num2str(NumModalStates));
    set_param(aswing_block_path, 'NumPhysicalStates', num2str(NumPhysicalStates));

    fprintf('\n[WL_main] AswingPlant block configured:\n');
    fprintf('Block path        : %s\n', aswing_block_path);
    fprintf('NumModalStates    : %d\n', NumModalStates);
    fprintf('NumPhysicalStates : %d\n', NumPhysicalStates);

    %% --- Configure Simulink Solver ---
    set_param(model_name, 'FixedStep', num2str(Dt_sim));

    fprintf('\n[WL_main] Simulink model configured:\n');
    fprintf('Model name : %s\n', model_name);
    fprintf('Model file : %s\n', model_file);
    fprintf('Fixed step : %g s\n', Dt_sim);

    fprintf('\n==================================================\n');
    fprintf('Setup complete. Starting Simulink simulation...\n');
    fprintf('==================================================\n\n');
end