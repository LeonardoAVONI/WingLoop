classdef AswingPlant < matlab.System
%ASWINGPLANT  WingLoop Simulink V2 plant block: the ASWING aircraft, simulated by WingLoop.
%
%   Copyright (c) 2026 Leonardo Avoni. Simulink V2 method: Matteo Alfonso Incarbone.
%   Part of WingLoop (CC BY-NC-SA 4.0).
%
%   Output  state : ASWING state vector y_k at the current Simulink time t_k
%   Input   u     : commands u_k (order = CommandNames), sent back to WingLoop
%
%   No direct feedthrough: outputImpl returns y_k (received from WingLoop), the
%   controller computes u_k = C(y_k), and updateImpl sends u_k. This gives the same
%   timing as the other WingLoop backends and no algebraic loop.
%
%   Must run in interpreted execution (tcpclient). WingLoop builds and configures
%   the block itself (wingloop_v2_make_model.m, wingloop_v2_run.m).

    properties (Nontunable)
        Host         = '127.0.0.1'
        Port         = 5005
        NumStates    = 1
        CommandNames = 'F1'      % comma-separated, e.g. 'F1,F2,F3,F4,E1,E2'
        SampleTime   = 0.01
        Timeout      = 1200      % [s] per message (a hard ASWING step can take minutes)
        ConnectTries = 120       % 0.5 s apart
    end

    properties (Access = private)
        client
        y
        needState = true
        nCmd = 1
    end

    methods
        function obj = AswingPlant(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            names = strtrim(strsplit(obj.CommandNames, ','));
            names = names(~cellfun(@isempty, names));
            obj.nCmd = numel(names);
            ok = false;
            lastErr = '';
            for k = 1:obj.ConnectTries
                try
                    obj.client = tcpclient(obj.Host, double(obj.Port), 'Timeout', double(obj.Timeout), ...
                        'ConnectTimeout', 5);
                    ok = true;
                    break;
                catch e
                    lastErr = e.message;
                    pause(0.5);
                end
            end
            if ~ok
                error('WingLoop:connect', 'AswingPlant: no WingLoop server on %s:%d (%s)', ...
                    obj.Host, obj.Port, lastErr);
            end
            nameBytes = uint8(strjoin(names, ','));
            write(obj.client, [typecast(uint32(hex2dec('574C5632')), 'uint8'), ...
                typecast(uint32(obj.nCmd), 'uint8'), ...
                typecast(uint32(numel(nameBytes)), 'uint8'), nameBytes]);
            obj.y = zeros(obj.NumStates, 1);
            obj.needState = true;
        end

        function y = outputImpl(obj, ~)        % u is not used: no direct feedthrough
            if obj.needState
                kind = typecast(uint8(read(obj.client, 4, 'uint8')), 'uint32');
                if kind == 0
                    error('WingLoop:stop', 'AswingPlant: simulation stopped by WingLoop');
                elseif kind ~= 1
                    error('WingLoop:protocol', 'AswingPlant: unexpected message %d', kind);
                end
                read(obj.client, 1, 'double');                 % WingLoop time (not used)
                obj.y = reshape(double(read(obj.client, obj.NumStates, 'double')), [], 1);
                obj.needState = false;
            end
            y = obj.y;
        end

        function updateImpl(obj, u)
            u = double(u(:));
            if numel(u) ~= obj.nCmd
                error('WingLoop:size', 'AswingPlant: %d commands, %d names', numel(u), obj.nCmd);
            end
            write(obj.client, [typecast(uint32(2), 'uint8'), typecast(u', 'uint8')]);
            obj.needState = true;
        end

        function releaseImpl(obj)
            if ~isempty(obj.client)
                clear obj.client;
                obj.client = [];
            end
        end

        function flag = isInputDirectFeedthroughImpl(~, ~)
            flag = false;
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.SampleTime);
        end

        function num = getNumInputsImpl(~)
            num = 1;
        end

        function num = getNumOutputsImpl(~)
            num = 1;
        end

        function s = getOutputSizeImpl(obj)
            s = [obj.NumStates 1];
        end

        function d = getOutputDataTypeImpl(~)
            d = 'double';
        end

        function c = isOutputComplexImpl(~)
            c = false;
        end

        function f = isOutputFixedSizeImpl(~)
            f = true;
        end
    end
end
