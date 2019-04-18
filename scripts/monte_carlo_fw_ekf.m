% Prepare Variables for Plotting
clear, close all
format compact
set(0,'DefaultFigureWindowStyle','normal')
addpath('../lib/yamlmatlab')
params = ReadYaml('../params/fixed_wing1.yaml');
skip = 500;
fig_width = 650;
fig_x_scale = 1;
fig_y_scale = 1;
num_runs = 100;


% Plot Error in Position, Velocity, and Attitude
fig = figure('Renderer', 'painters', 'Position', [10 10 fig_width fig_width*3/3]);
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_x_scale*fig_pos(3) fig_y_scale*fig_pos(4)];
for j = 1:num_runs
    % Load data and compute error states
    truth = reshape(fread(fopen(strcat(['/tmp/fw_monte_carlo_ekf_true_',int2str(j),'.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;euler;acc_bias;gyro_bias;wind_inertial;baro_bias]
    est = reshape(fread(fopen(strcat(['/tmp/fw_monte_carlo_ekf_est_',int2str(j),'.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;euler;acc_bias;gyro_bias;wind_inertial;baro_bias]
%     cov = reshape(fread(fopen(strcat(['/tmp/fw_monte_carlo_ekf_cov_',int2str(j),'.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;att;acc_bias;gyro_bias;wind_inertial;baro_bias]
    error = truth - est;
    error(10,:) = wrapToPi(truth(10,:) - est(10,:));
    
    subplot(4, 1, 1), hold on, grid on
    plot(truth(1, 1:skip:end), vecnorm(error(2:4, 1:skip:end)), 'linewidth', 1.0)
    ylabel('Position (m)')
    axis tight
    
    subplot(4, 1, 2), hold on, grid on
    plot(truth(1, 1:skip:end), vecnorm(error(5:7, 1:skip:end)), 'linewidth', 1.0)
    ylabel('Velocity (m/s)')
    axis tight
    
    subplot(4, 1, 3), hold on, grid on
    plot(truth(1, 1:skip:end), vecnorm(error(8:9, 1:skip:end)), 'linewidth', 1.0)
    ylabel('Roll/Pitch (rad)')
    xlabel('Time (sec)')
    axis tight
    
    subplot(4, 1, 4), hold on, grid on
    plot(truth(1, 1:skip:end), error(10, 1:skip:end), 'linewidth', 1.0)
    ylabel('Heading (rad)')
    xlabel('Time (sec)')
    axis tight
end
set(gcf, 'name', 'Pos/Vel/Att Error', 'NumberTitle', 'off')
set(gcf, 'color', 'w')
print(gcf, '~/Dropbox/dev/fw_lqr_paper/figures/ekf_pos_vel_att','-dpdf')
!pdfcrop ~/Dropbox/dev/fw_lqr_paper/figures/ekf_pos_vel_att.pdf ~/Dropbox/dev/fw_lqr_paper/figures/ekf_pos_vel_att.pdf


% Plot Error in Accel Bias, Gyro Bias, and Wind Velocity
fig = figure('Renderer', 'painters', 'Position', [10 10 fig_width fig_width*3/4]);
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_x_scale*fig_pos(3) fig_y_scale*fig_pos(4)];
for j = 1:num_runs
    % Load data and compute error states
    truth = reshape(fread(fopen(strcat(['/tmp/fw_monte_carlo_ekf_true_',int2str(j),'.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;euler;acc_bias;gyro_bias;wind_inertial;baro_bias]
    est = reshape(fread(fopen(strcat(['/tmp/fw_monte_carlo_ekf_est_',int2str(j),'.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;euler;acc_bias;gyro_bias;wind_inertial;baro_bias]
%     cov = reshape(fread(fopen(strcat(['/tmp/fw_monte_carlo_ekf_cov_',int2str(j),'.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;att;acc_bias;gyro_bias;wind_inertial;baro_bias]
    error = truth - est;
    
    subplot(3, 1, 1), hold on, grid on
    plot(truth(1, 1:skip:end), vecnorm(error(11:13, 1:skip:end)), 'linewidth', 1.0)
    ylabel('Accel Bias (m/s^2)')
    axis tight
    
    subplot(3, 1, 2), hold on, grid on
    plot(truth(1, 1:skip:end), vecnorm(error(14:16, 1:skip:end)), 'linewidth', 1.0)
    ylabel('Gyro Bias (rad/s)')
    axis tight
    
    subplot(3, 1, 3), hold on, grid on
    plot(truth(1, 1:skip:end), vecnorm(error(17:19, 1:skip:end)), 'linewidth', 1.0)
    ylabel('Wind Velocity (m/s)')
    xlabel('Time (sec)')
    axis tight
end
set(gcf, 'name', 'Accel/Gyro/Wind Error', 'NumberTitle', 'off')
set(gcf, 'color', 'w')
print(gcf, '~/Dropbox/dev/fw_lqr_paper/figures/ekf_acc_gyro_wind','-dpdf')
!pdfcrop ~/Dropbox/dev/fw_lqr_paper/figures/ekf_acc_gyro_wind.pdf ~/Dropbox/dev/fw_lqr_paper/figures/ekf_acc_gyro_wind.pdf
