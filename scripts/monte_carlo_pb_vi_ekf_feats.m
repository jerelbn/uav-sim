% Prepare Variables for Plotting
clear, close all
format compact
set(0,'DefaultFigureWindowStyle','normal')
addpath('../lib/yamlmatlab')
params = ReadYaml('../params/pb_vi_ekf_params.yaml');
skip = 5;
fig_width = 650;
fig_x_scale = 1;
fig_y_scale = 1;
num_runs = 100;
save_plots = true;

if params.use_drag
    nbd = 16; % number of base degrees of freedom
else
    nbd = 15;
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
    plot(truth(1, 1:skip:end), abs(error(13, 1:skip:end)), 'linewidth', 1.0)
    ylabel('Accel Z Bias (m/s^2)')
    axis tight
    
    subplot(3, 1, 2), hold on, grid on
    plot(truth(1, 1:skip:end), vecnorm(error(14:16, 1:skip:end)), 'linewidth', 1.0)
    ylabel('Gyro Bias (rad/s)')
    axis tight
    
    subplot(3, 1, 3), hold on, grid on
    plot(truth(1, 1:skip:end), abs(error(20, 1:skip:end)), 'linewidth', 1.0)
    ylabel('Feature 1 Depth')
    xlabel('Time (sec)')
    axis tight
end
set(gcf, 'name', 'Accel/Gyro/Drag Error', 'NumberTitle', 'off')
set(gcf, 'color', 'w')
if save_plots == true
    print(gcf, '~/Dropbox/dev/vi_ekf_paper/figures/ekf_acc_gyro_depth','-dpdf')
    !pdfcrop ~/Dropbox/dev/vi_ekf_paper/figures/ekf_acc_gyro_depth.pdf ~/Dropbox/dev/vi_ekf_paper/figures/ekf_acc_gyro_depth.pdf
end
