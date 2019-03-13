% Prepare Variables for Plotting
clear, close all
format compact
set(0,'DefaultFigureWindowStyle','docked')
addpath('yamlmatlab')
wing1_params = ReadYaml('../params/fixed_wing1.yaml');

plot_environment('wing1')
plot_vehicle('wing1') 
plot_sensors('wing1', wing1_params)
plot_ekf('wing1', true)
% plot_bicycle('bike1')
% animate_3d(100, 'wing1');
% animate_img(2,'quad1');