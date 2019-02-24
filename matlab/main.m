%% Prepare Variables for Plotting
clear
format compact
set(0,'DefaultFigureWindowStyle','docked')
f = 1; % Starts figure numbering
plot_covariance = 1;
cam_max_feat = 10000;

%% Run Simulation
% !cd ../build && ./uav_sim

%% Read logs
directory = '/tmp/';
read_logs;

%% Plot environment
plot_environment;

%% Plot states
plot_states;
% plot_target_states;
% plot_bicycle_states;

%% Plot commands
% plot_commands;
% plot_bicycle_commands;

%% Plot sensor measurements
% plot_imu;

%% Plot 3D animation
% animate_3d(100, env, true_state, command, bicycle_state);
% animate_img(1, pix);