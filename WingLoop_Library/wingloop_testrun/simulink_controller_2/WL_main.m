clc;
close all;
clear;

%% ========================================================================
%  USER CONFIGURATION
%  Change only this section when using a different aircraft, trim point,
%  or simulation setup.
%  ========================================================================

%% --- Time Configuration ---
T_sim      = 10;     % Total simulation time [s]
Dt_asw     = 0.1;    % ASWING step size [s]
Dt_sim     = 0.1;    % Simulink solver step size [s]
Ts_control = 0.1;    % Controller sample time [s]
tau        = 0.1;    % Filter time constant
t_int      = 0.5;    % Integral time constant / user-defined parameter

%% --- ASWING Case Files ---
% Geometry file must be located in:
% WingLoop/WingLoop_Library/wingloop_testrun/Geometries/
ASW_FILE = 't_tail_HALE.asw';

% These files must be located in:
% WingLoop/WingLoop_Library/wingloop_testrun/aswing_geometry/
PNT_FILE   = 't_tail_HALE.pnt';
SET_FILE   = 't_tail_HALE.set';
STATE_FILE = 't_tail_HALE.state';
GUST_FILE  = 'gust_H40.gust';

%% --- Aircraft / Model Dimensions ---
% Change these values when using a different aircraft model.
ROM.n_modal      = 91;    % Number of modal states
FullModel.n_orig = 1882;  % Number of original physical states
FullModel.n_in   = 6;     % Number of control inputs

%% --- Trim Control Inputs ---
% Change these values when using a different trim point.
F2ref = -6.63806152;
F3ref = 4.54747351E-12;
E2ref = 23.6475887;

u_trim = [0, F2ref, F3ref, 0, E2ref, E2ref];

%% --- Simulink Model ---
model_name = 'WL_test';
model_file = 'WL_test.slx';

%% ========================================================================
%  AUTOMATIC SETUP AND SIMULATION
%  Do not modify this section unless you are changing the WingLoop interface.
%  ========================================================================

run_wingloop_simulink_setup( ...
    T_sim, Dt_asw, Dt_sim, Ts_control, tau, t_int, ...
    ASW_FILE, PNT_FILE, SET_FILE, STATE_FILE, GUST_FILE, ...
    ROM, FullModel, u_trim, ...
    model_name, model_file ...
);

sim(model_file);




