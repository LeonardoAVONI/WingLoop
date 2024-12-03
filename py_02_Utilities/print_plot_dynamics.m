function print_plot_dynamics()

    %% Import the data

    TIME = evalin('base', 'TIME');

    EarthX = evalin('base', 'EarthX');
    EarthY = evalin('base', 'EarthY');
    EarthZ = evalin('base', 'EarthZ');

    PITCH = evalin('base', 'PITCH');
    ROLL = evalin('base', 'ROLL');
    YAW = evalin('base', 'YAW');

    ALPHA = evalin('base', 'ALPHA');
    BETA = evalin('base', 'BETA');
    VELOCITY = evalin('base', 'VELOCITY');

    %% Print the data
    hold on
    plot(TIME,PITCH)
    plot(TIME,EarthZ)
    hold off

    %% Plot the data
    input("continue")

end
