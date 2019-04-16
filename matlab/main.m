% Prepare Variables for Plotting
clear, close all
format compact
set(0,'DefaultFigureWindowStyle','docked')
addpath('../lib/yamlmatlab')
sim_params = ReadYaml('../params/sim.yaml');
wing1_params = ReadYaml('../params/fixed_wing1.yaml');
quad1_params = ReadYaml('../params/pb_vi_ekf_params.yaml');

plot_environment('wing1', sim_params, quad1_params)
% plot_vehicle(wing1_params)
% plot_sensors(wing1_params)
plot_fw_ekf('wing1', true)
% animate_3d(200, 'wing1', 10)

% plot_environment('quad1', sim_params, quad1_params)
% plot_vehicle(quad1_params)
% plot_sensors(quad1_params)
% plot_quad_ekf(quad1_params, true, false, false)
% animate_3d(50, 'quad1', 2)
% animate_img(1,quad1_params,)

% plot_bicycle('bike1')
