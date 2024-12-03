function append_flight_data(input_instantaneous_struct)
    %disp(input_instantaneous_struct)
    if not(evalin('base', 'exist(''PITCH'', ''var'')'))
        % Initialize if they don't exist
        assignin('base', 'TIME', input_instantaneous_struct.Time);

        assignin('base', 'EarthX', input_instantaneous_struct.earthX);
        assignin('base', 'EarthY', input_instantaneous_struct.earthY);
        assignin('base', 'EarthZ', input_instantaneous_struct.earthZ);

        assignin('base', 'PITCH', input_instantaneous_struct.Pitch);
        assignin('base', 'ROLL', input_instantaneous_struct.Bank);
        assignin('base', 'YAW', input_instantaneous_struct.Heading);

        assignin('base', 'ALPHA', input_instantaneous_struct.Alpha);
        assignin('base', 'BETA', input_instantaneous_struct.Beta);
        assignin('base', 'VELOCITY', input_instantaneous_struct.Velocity);

    else
        % Append new values if they do exist
        assignin('base', 'TIME', [evalin('base', 'TIME'), input_instantaneous_struct.Time]);

        assignin('base', 'EarthX', [evalin('base', 'EarthX'), input_instantaneous_struct.earthX]);
        assignin('base', 'EarthY', [evalin('base', 'EarthY'), input_instantaneous_struct.earthY]);
        assignin('base', 'EarthZ', [evalin('base', 'EarthZ'), input_instantaneous_struct.earthZ]);

        assignin('base', 'PITCH', [evalin('base', 'PITCH'), input_instantaneous_struct.Pitch]);
        assignin('base', 'ROLL', [evalin('base', 'ROLL'), input_instantaneous_struct.Bank]);
        assignin('base', 'YAW', [evalin('base', 'YAW'), input_instantaneous_struct.Heading]);

        assignin('base', 'ALPHA', [evalin('base', 'ALPHA'), input_instantaneous_struct.Alpha]);
        assignin('base', 'BETA', [evalin('base', 'BETA'), input_instantaneous_struct.Beta]);
        assignin('base', 'VELOCITY', [evalin('base', 'VELOCITY'), input_instantaneous_struct.Velocity]);  

    end
end
