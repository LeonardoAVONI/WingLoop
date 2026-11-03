function wingloop_v2_run(handshake_file)
%WINGLOOP_V2_RUN  Run a WingLoop Simulink V2 simulation (one sim() for the whole run).
%
%   wingloop_v2_run(handshake_file)
%
%   handshake_file: JSON written by WingLoop (SimulinkV2.py) with port, host, n_state,
%   Dt, stop_time, model, model_dir, show_simulink, run_user_controller, precomputed,
%   timeout. Called by WingLoop through the MATLAB engine (Linux), or by
%   wingloop_v2_windows_launcher.m (Windows + WSL, untested).
%
%   Same rules as Simulink V1: the model's solver is forced to fixed-step discrete
%   with step Dt, every block with a positive sample time gets Dt, and Nfilter = 1/Dt
%   is pushed to the base workspace. UserController.m (if run_user_controller) is
%   instantiated before sim, so its assignin('base', ...) values are available.
%
%   Copyright (c) 2026 Leonardo Avoni. Simulink V2 method: Matteo Alfonso Incarbone.
%   Part of WingLoop (CC BY-NC-SA 4.0).

    hs = jsondecode(fileread(handshake_file));
    addpath(fileparts(mfilename('fullpath')));
    addpath(hs.model_dir);
    old = cd(hs.model_dir);
    cleanup = onCleanup(@() cd(old));

    if hs.run_user_controller && isfile(fullfile(hs.model_dir, 'UserController.m'))
        UserController(hs.precomputed); %#ok<NASGU> (constructor pushes base-workspace variables)
    end
    assignin('base', 'Nfilter', 1.0 / hs.Dt);
    assignin('base', 'WL_NumStates', hs.n_state);   % model rule: statein sized by WL_NumStates

    [wrapper, names] = wingloop_v2_make_model(hs.model, hs.model_dir);
    fprintf('[wingloop_v2_run] %s: %d commands (%s), %d states, Dt = %g s, stop %g s\n', ...
        wrapper, numel(names), strjoin(names, ','), hs.n_state, hs.Dt, hs.stop_time);

    plant = [wrapper '/AswingPlant'];
    set_param(plant, 'Host', hs.host, 'Port', num2str(hs.port), ...
        'NumStates', num2str(hs.n_state), 'SampleTime', num2str(hs.Dt, 17), ...
        'Timeout', num2str(hs.timeout));

    set_param(wrapper, 'Solver', 'FixedStepDiscrete', 'FixedStep', num2str(hs.Dt, 17), ...
        'StartTime', '0', 'StopTime', num2str(hs.stop_time, 17));
    blocks = find_system(wrapper, 'LookUnderMasks', 'all', 'Type', 'Block');
    for k = 1:numel(blocks)
        try
            st = str2double(get_param(blocks{k}, 'SampleTime'));
            if st > 0
                set_param(blocks{k}, 'SampleTime', num2str(hs.Dt, 17));
            end
        catch
        end
    end
    if hs.show_simulink
        open_system(wrapper);
    end

    try
        sim(wrapper);
    catch err
        % A stop requested by WingLoop (early end of the run) is a normal end.
        if ~contains(err.message, 'stopped by WingLoop') && ~any(strcmp(err.identifier, {'WingLoop:stop'}))
            rethrow(err);
        end
    end
end
