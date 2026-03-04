function output = UAV_control_Strategy_LQR(instantaneous_state, Dt)
    test = true;
    if test
        du = [0,0,0,0,10,10];
    else
        persistent K x_trim u_trim
        if isempty(K)
            load('LQR_gains.mat', 'K', 'x_trim', 'u_trim');  % or hard-code
        end
        
        dx = instantaneous_state(:) - x_trim(:);   % column vectors
        du = u_trim(:) - K * dx;
        
        % clipping exactly as in Python
        du(1:4) = max(min(du(1:4), 20), -20);
        du(5:6) = max(min(du(5:6), 10), -10);
  
    end
    % Return Python-friendly struct
    output = struct('F1',du(1),'F2',du(2),'F3',du(3),'F4',du(4), ...
                        'E1',du(5),'E2',du(6));
end