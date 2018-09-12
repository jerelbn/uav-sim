%% Prepare Variables for Plotting
clear
format compact
set(0,'DefaultFigureWindowStyle','docked')
f = 1; % Starts figure numbering
plot_covariance = 1;

%% Run Simulation
% !cd ../build && ./uav_sim

%% Read logs
directory = '../logs/';
read_logs;

%% Plot environment
plot_environment;

%% Plot states and wind
% plot_states;

%% Plot estimated target states
plot_target_states;

%% Plot bicycle states
% plot_bicycle_states;

%% Plot commands
% plot_commands;

%% Plot bicycle commands
% plot_bicycle_commands;

%% Plot sensor measurements
% plot_measurements;

%% Plot 3D animation
% animate_3d(5,env, true_state, ekf_state, command, bicycle_state);

%% Plot image animation
% animate_img(1,pix);