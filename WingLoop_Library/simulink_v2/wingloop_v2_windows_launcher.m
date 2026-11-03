function wingloop_v2_windows_launcher(wsl_command, handshake_file, varargin)
%WINGLOOP_V2_WINDOWS_LAUNCHER  Simulink V2 with MATLAB on Windows and WingLoop in WSL.
%
%   *** UNTESTED: WingLoop 3.0 is tested on Linux only. ***
%
%   wingloop_v2_windows_launcher(wsl_command, handshake_file)
%   wingloop_v2_windows_launcher(..., 'Distro', 'Ubuntu', 'Timeout', 600)
%
%   wsl_command    : shell command run inside WSL that starts your WingLoop script, whose
%                    controller uses simulink_mode="v2" and simulink_launch="external",
%                    e.g. 'cd ~/case && conda run -n wingloop python run_case.py'
%   handshake_file : Windows path of the handshake JSON that this WingLoop script writes
%                    (Launch_WingLoop_Control(..., simulink_handshake_file=...)), e.g.
%                    '\\wsl.localhost\Ubuntu\home\me\case\wingloop_v2_handshake.json'
%
%   The launcher starts the WSL script in a new console, waits for the handshake file,
%   rewrites its model folder to the Windows view of the WSL folder (when it is a WSL
%   path), then runs wingloop_v2_run. WSL2 forwards localhost, so AswingPlant reaches
%   the WingLoop server at 127.0.0.1. The controller model (.slx) and UserController.m
%   must be readable from Windows (a WSL folder seen through \\wsl.localhost works).
%
%   Copyright (c) 2026 Leonardo Avoni. Simulink V2 method: Matteo Alfonso Incarbone,
%   whose original launcher (WL_init_callback.m, WingLoop_V3.0 fork) this replaces.
%   Part of WingLoop (CC BY-NC-SA 4.0).

    p = inputParser;
    addParameter(p, 'Distro', '');
    addParameter(p, 'Timeout', 600);
    parse(p, varargin{:});
    distro = p.Results.Distro;

    if isfile(handshake_file)
        delete(handshake_file);
    end
    if isempty(distro)
        wsl = 'wsl';
    else
        wsl = sprintf('wsl -d %s', distro);
    end
    cmd = sprintf('start "WingLoop" %s -e bash -lc "%s"', wsl, strrep(wsl_command, '"', '\"'));
    fprintf('[WingLoop] %s\n', cmd);
    system(cmd);

    t0 = tic;
    while ~isfile(handshake_file)
        if toc(t0) > p.Results.Timeout
            error('WingLoop:launch', 'No handshake file after %g s: %s', p.Results.Timeout, handshake_file);
        end
        pause(0.5);
    end
    pause(0.5);

    hs = jsondecode(fileread(handshake_file));
    if startsWith(hs.model_dir, '/')
        [root, ~] = regexp(handshake_file, '^(\\\\wsl[^\\]*\\[^\\]+)', 'tokens', 'match');
        if ~isempty(root)
            hs.model_dir = [root{1}{1} strrep(hs.model_dir, '/', '\')];
        end
    end
    hs.host = '127.0.0.1';
    local = [tempname '.json'];
    fid = fopen(local, 'w');
    fwrite(fid, jsonencode(hs));
    fclose(fid);
    wingloop_v2_run(local);
end
