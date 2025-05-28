function route = planned_path()

    TIME = evalin('base', 'TIME');
    % declare other base workspace variables if you need them for path
    % creation

    route.earthX_planned=NaN;
    route.earthY_planned=NaN;
    route.earthZ_planned=NaN;

    route.PITCH_planned=0.8877;
    route.ROLL_planned=NaN;
    route.YAW_planned=NaN;

    route.ALPHA_planned=NaN;
    route.BETA_planned=NaN;
    route.VELOCITY_planned=NaN;

end
