% Prepare Variables for Plotting
clear, close all
format compact
set(0,'DefaultFigureWindowStyle','normal')
addpath('../lib/yamlmatlab')
params = ReadYaml('../params/pb_vi_ekf_params.yaml');
skip = 500;
fig_width = 650;
fig_x_scale = 1;
fig_y_scale = 1;
num_runs = 10;
save_plots = false;

if params.use_drag
    nbd = 16; % number of base degrees of freedom
else
    nbd = 15;
end


% Plot Error in Position, Velocity, and Attitude
fig = figure('Renderer', 'painters', 'Position', [10 10 fig_width fig_width*3/3]);
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_x_scale*fig_pos(3) fig_y_scale*fig_pos(4)];
for j = 1:num_runs
    % Load data and compute error states
    truth = reshape(fread(fopen(strcat(['/tmp/monte_carlo_pb_vi_ekf_true_',int2str(j),'.log']), 'r'), 'double'), 1 + nbd + 3 * params.num_features, []); % [time;pos;vel;euler;acc_bias;gyro_bias;drag;pix;rho]
    est = reshape(fread(fopen(strcat(['/tmp/monte_carlo_pb_vi_ekf_est_',int2str(j),'.log']), 'r'), 'double'), 1 + nbd + 3 * params.num_features, []); % [time;pos;vel;euler;acc_bias;gyro_bias;drag;pix;rho]
%     cov = reshape(fread(fopen(strcat(['/tmp/monte_carlo_pb_vi_ekf_cov_',int2str(j),'.log']), 'r'), 'double'), 1 + nbd + 3 * params.num_features, []); % [time;pos;vel;euler;acc_bias;gyro_bias;drag;pix;rho]
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
if save_plots == true
    print(gcf, '~/Dropbox/dev/vi_ekf_paper/figures/ekf_pos_vel_att','-dpdf')
    !pdfcrop ~/Dropbox/dev/vi_ekf_paper/figures/ekf_pos_vel_att.pdf ~/Dropbox/dev/vi_ekf_paper/figures/ekf_pos_vel_att.pdf
end


% Plot Error in Accel Bias, Gyro Bias, and Wind Velocity
fig = figure('Renderer', 'painters', 'Position', [10 10 fig_width fig_width*3/4]);
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_x_scale*fig_pos(3) fig_y_scale*fig_pos(4)];
for j = 1:num_runs
    % Load data and compute error states
    truth = reshape(fread(fopen(strcat(['/tmp/monte_carlo_pb_vi_ekf_true_',int2str(j),'.log']), 'r'), 'double'), 1 + nbd + 3 * params.num_features, []); % [time;pos;vel;euler;acc_bias;gyro_bias;drag;pix;rho]
    est = reshape(fread(fopen(strcat(['/tmp/monte_carlo_pb_vi_ekf_est_',int2str(j),'.log']), 'r'), 'double'), 1 + nbd + 3 * params.num_features, []); % [time;pos;vel;euler;acc_bias;gyro_bias;drag;pix;rho]
%     cov = reshape(fread(fopen(strcat(['/tmp/monte_carlo_pb_vi_ekf_cov_',int2str(j),'.log']), 'r'), 'double'), 1 + nbd + 3 * params.num_features, []); % [time;pos;vel;euler;acc_bias;gyro_bias;drag;pix;rho]
    error = truth - est;
    
    subplot(3, 1, 1), hold on, grid on
    plot(truth(1, 1:skip:end), vecnorm(error(11:13, 1:skip:end)), 'linewidth', 1.0)
    ylabel('Accel Bias (m/s^2)')
    axis tight
    
    subplot(3, 1, 2), hold on, grid on
    plot(truth(1, 1:skip:end), vecnorm(error(14:16, 1:skip:end)), 'linewidth', 1.0)
    ylabel('Gyro Bias (rad/s)')
    axis tight
    
%     subplot(3, 1, 3), hold on, grid on
%     plot(truth(1, 1:skip:end), vecnorm(error(17:19, 1:skip:end)), 'linewidth', 1.0)
%     ylabel('Wind Velocity (m/s)')
%     xlabel('Time (sec)')
%     axis tight
end
set(gcf, 'name', 'Accel/Gyro/Wind Error', 'NumberTitle', 'off')
set(gcf, 'color', 'w')
if save_plots == true
    print(gcf, '~/Dropbox/dev/vi_ekf_paper/figures/ekf_acc_gyro_lm','-dpdf')
    !pdfcrop ~/Dropbox/dev/vi_ekf_paper/figures/ekf_acc_gyro_lm.pdf ~/Dropbox/dev/vi_ekf_paper/figures/ekf_acc_gyro_lm.pdf
end
