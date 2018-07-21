%% Prepare Variables for Plotting
clear
format compact
set(0,'DefaultFigureWindowStyle','docked')
f = 1; % Starts figure numbering
plot_covariance = 1;

% %% Run Simulation
% !cd ../build && ./uav_sim

%% Read logs
directory = '../logs/';
read_logs;

%% Plot states
plot_states;