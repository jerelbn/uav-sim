% Prepare Variables for Plotting
clear, close all
format compact
set(0,'DefaultFigureWindowStyle','docked')

plot_environment('wing1')
plot_vehicle('wing1') 
plot_sensors('wing1')
plot_ekf('wing1', true)
% plot_bicycle('bike1')
% animate_3d(100, 'wing1');
% animate_img(2,'quad1');