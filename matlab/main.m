% Prepare Variables for Plotting
clear, close all
format compact
set(0,'DefaultFigureWindowStyle','docked')

plot_environment('wing1')
plot_vehicle('wing1', true)
% plot_bicycle('bike1')
% animate_3d(50, 'wing1');
% animate_img(2,'quad1');