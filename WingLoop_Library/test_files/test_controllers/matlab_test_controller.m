function output = UAV_control_Strategy_LQR(instantaneous_state, Dt)
    %{
    Very simple python control file:
        takes the input state
        takes the first 4 elements of the state and associates them with flap deflections F1 to F4
        takes the last 2 elements of du and associates them with E1 and E2
    This controller is to be used with an aircraft with 6 control inputs
    %}

    du = [0,0,0,0,10,10];
    testvalue = instantaneous_state(1:4);

    % Return Python-friendly struct
    output = struct('F1',du(1),'F2',du(2),'F3',du(3),'F4',du(4), ...
                        'E1',du(5),'E2',du(6));
    output = struct('F1',testvalue(1),'F2',testvalue(2),'F3',testvalue(3),'F4',testvalue(4), ...
                        'E1',du(5),'E2',du(6));
end