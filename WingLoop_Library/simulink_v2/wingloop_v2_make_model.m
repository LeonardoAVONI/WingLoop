function [wrapper, names] = wingloop_v2_make_model(model, model_dir)
%WINGLOOP_V2_MAKE_MODEL  Build the Simulink V2 wrapper of a WingLoop controller model.
%
%   [wrapper, names] = wingloop_v2_make_model(model, model_dir)
%
%   model     : name of a WingLoop controller model (same rules as Simulink V1 / FMU:
%               one root Inport "statein" receiving the state vector, one root Outport
%               per command named F1..F20 / E1..E20)
%   model_dir : folder of model.slx; the wrapper "<model>_wlv2.slx" is written there
%
%   In the wrapper, "statein" is replaced by the AswingPlant block output and the
%   command Outports by a Mux into the AswingPlant input. Returns the wrapper name and
%   the command names (Outport order). The wrapper is rebuilt when model.slx is newer.
%
%   Copyright (c) 2026 Leonardo Avoni. Simulink V2 method: Matteo Alfonso Incarbone.
%   Part of WingLoop (CC BY-NC-SA 4.0).

    src = fullfile(model_dir, [model '.slx']);
    wrapper = [model '_wlv2'];
    dst = fullfile(model_dir, [wrapper '.slx']);
    if ~isfile(src)
        error('WingLoop:model', 'Controller model not found: %s', src);
    end

    rebuild = ~isfile(dst);
    if ~rebuild
        s = dir(src); d = dir(dst);
        rebuild = s.datenum > d.datenum;
    end

    if bdIsLoaded(wrapper)
        close_system(wrapper, 0);
    end
    if ~rebuild
        load_system(dst);
        names = strtrim(strsplit(get_param([wrapper '/AswingPlant'], 'CommandNames'), ','));
        names = strrep(names, '''', '');
        return;
    end

    if bdIsLoaded(model)
        close_system(model, 0);
    end
    copyfile(src, dst, 'f');
    load_system(dst);

    inp = find_system(wrapper, 'SearchDepth', 1, 'BlockType', 'Inport', 'Name', 'statein');
    if isempty(inp)
        error('WingLoop:model', '%s has no root Inport named "statein"', model);
    end
    outs = find_system(wrapper, 'SearchDepth', 1, 'BlockType', 'Outport');
    ports = cellfun(@(b) str2double(get_param(b, 'Port')), outs);
    [~, order] = sort(ports);
    outs = outs(order);
    names = cellfun(@(b) get_param(b, 'Name'), outs, 'UniformOutput', false);
    if isempty(names)
        error('WingLoop:model', '%s has no root Outports (commands)', model);
    end

    % plant block
    pos_in = get_param(inp{1}, 'Position');
    plant = [wrapper '/AswingPlant'];
    add_block('simulink/User-Defined Functions/MATLAB System', plant, ...
        'System', 'AswingPlant', 'Position', pos_in + [-150 -10 -50 30]);
    set_param(plant, 'SimulateUsing', 'Interpreted execution');
    set_param(plant, 'CommandNames', strjoin(names, ','));

    % statein -> plant output
    lh = get_param(inp{1}, 'LineHandles');
    dsts = [];
    if lh.Outport > 0
        dsts = get_param(lh.Outport, 'DstPortHandle');
        delete_line(lh.Outport);
    end
    delete_block(inp{1});
    ph = get_param(plant, 'PortHandles');
    for k = 1:numel(dsts)
        add_line(wrapper, ph.Outport(1), dsts(k), 'autorouting', 'on');
    end

    % command outports -> mux -> plant input
    pos_out = get_param(outs{1}, 'Position');
    mux = [wrapper '/WingLoop_Commands'];
    add_block('simulink/Signal Routing/Mux', mux, 'Inputs', num2str(numel(outs)), ...
        'Position', [pos_out(1)+80, pos_out(2), pos_out(1)+85, pos_out(2)+20*numel(outs)]);
    mh = get_param(mux, 'PortHandles');
    for k = 1:numel(outs)
        lh = get_param(outs{k}, 'LineHandles');
        if lh.Inport <= 0
            error('WingLoop:model', 'Outport %s is not connected', names{k});
        end
        srcp = get_param(lh.Inport, 'SrcPortHandle');
        delete_line(lh.Inport);
        delete_block(outs{k});
        add_line(wrapper, srcp, mh.Inport(k), 'autorouting', 'on');
    end
    add_line(wrapper, mh.Outport(1), ph.Inport(1), 'autorouting', 'on');

    save_system(wrapper, dst);
end
