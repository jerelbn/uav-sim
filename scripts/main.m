% Prepare Variables for Plotting
clear, close all
format compact
set(0,'DefaultFigureWindowStyle','docked')
addpath('../lib/yamlmatlab')
sim_params = ReadYaml('../params/sim.yaml');
wing1_params = ReadYaml('../params/fixed_wing1.yaml');
quad1_params = ReadYaml('../params/quadrotor1.yaml');
gimbal_params = ReadYaml('../params/gimbal.yaml');
pbviekf_params = ReadYaml('../params/pb_vi_ekf_params.yaml');

% plot_environment('wing1', sim_params, quad1_params)
% plot_vehicle(wing1_params)
% plot_sensors(wing1_params)
% plot_fw_ekf('wing1', true)
% animate_3d(200, 'wing1', 10)

% plot_environment('quad1', sim_params, pbviekf_params)
% plot_vehicle(quad1_params)
plot_gimbal(gimbal_params)
plot_sensors(gimbal_params)
% plot_quad_ekf(pbviekf_params, true, true, false)
% animate_3d(50, 'quad1', 0.5)
% animate_img(1,quad1_params,)

% plot_bicycle('bike1')
