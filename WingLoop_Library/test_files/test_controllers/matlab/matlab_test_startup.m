% startup.m

% Assume x_state_trimmed is already in workspace (set by PyControl)
n_states = length(x_state_trimmed);
n_modes  = 32;
n_inputs = 6;

% Example (replace with real computation)
W_T_M = randn(n_modes, n_states) * 0.01;
W_T_M(1,1:10) = 1;

K_x = randn(n_inputs, n_states) * 0.005;
K_x(:,end-5:end) = -0.8 * eye(n_inputs);

q_state_trimmed = W_T_M * x_state_trimmed;

% These variables will stay in the base workspace
% PyControl will use them when calling UAV_control_Strategy_LQR etc.