function output = UAV_control_Strategy_LQR(instantaneous_state, Dt)

    disp("[matlab controller] workspace_scalar",workspace_scalar) 
    disp("[matlab controller] workspace_string",workspace_string)
    disp("[matlab controller] workspace_matrix",workspace_matrix)

    testvalue = instantaneous_state(1:6);

    output = struct('F1',testvalue(1),'F2',testvalue(2),'F3',testvalue(3),'F4',testvalue(4), ...
                        'E1',testvalue(5),'E2',testvalue(6));
end