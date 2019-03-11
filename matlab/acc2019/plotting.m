% Prepare Variables for Plotting
clear, close all
format compact
set(0,'DefaultFigureWindowStyle','normal')
skip = 20;
addpath '~/Dropbox/dev/code/matlab/YAMLMatlab'
params = ReadYaml('../../params/params.yaml');
fig_width = 600;
fig_x_scale = 0.88;
fig_y_scale = 0.95;

% Read binary files
copy_binaries(params.bearing_only);
directory = 'binaries/';
read_logs;

% Compute relative states for plotting
compute_relative;

% Bearings-only
if params.bearing_only == true
    
    % Relative position
    fig = figure('Renderer', 'painters', 'Position', [10 10 fig_width fig_width*2/3]);
    fig.PaperPositionMode = 'auto';
    fig_pos = fig.PaperPosition;
    fig.PaperSize = [fig_x_scale*fig_pos(3) fig_y_scale*fig_pos(4)];
    set(gcf, 'name', 'bop', 'NumberTitle', 'off');
    set(gcf,'color','w');
    ylabels = ["p_x", "p_y", "p_z"];
    for i=1:3
        subplot(3, 1, i); hold on; grid on;
        ylabel(ylabels(i));
        plot(target_truth_bo(1,1:skip:end), target_truth_bo(i+1,1:skip:end), 'b', 'linewidth', 1.5);
        plot(target_est_bo(1,1:skip:end), target_est_bo(i+1,1:skip:end), 'r', 'linewidth', 1.25);
        if i == 1
            title('Bearings-only Relative Target Position')
            legend('truth', 'estimate','location','southeast')
        end
        if i == 3
            xlabel('Time (seconds)')
        end
    end
    print(gcf, '~/Dropbox/dev/acc_2019/figures/bo_pos','-dpdf')
    
    % Relative radius and altitude
    fig = figure('Renderer', 'painters', 'Position', [10 10 fig_width fig_width*2/3]);
    fig.PaperPositionMode = 'auto';
    fig_pos = fig.PaperPosition;
    fig.PaperSize = [fig_x_scale*fig_pos(3) fig_y_scale*fig_pos(4)];
    set(gcf, 'name', 'borh', 'NumberTitle', 'off');
    set(gcf,'color','w');
    ylabels = ["r", "h"];
    for i=1:2
        subplot(2, 1, i); hold on; grid on;
        ylabel(ylabels(i));
        plot(target_truth_bo(1,1:skip:end), rh_true_bo(i,1:skip:end), 'b', 'linewidth', 1.5);
        plot(target_est_bo(1,1:skip:end), rh_est_bo(i,1:skip:end), 'r', 'linewidth', 1.25);
        if i == 1
            title('Bearings-only Relative Target Radius and Altitude')
            legend('truth', 'estimate','location','southeast')
        end
        if i == 2
            xlabel('Time (seconds)')
        end
    end
    print(gcf, '~/Dropbox/dev/acc_2019/figures/bo_rh','-dpdf')
    
    % Commanded and true velocity
    fig = figure('Renderer', 'painters', 'Position', [10 10 fig_width fig_width*2/3]);
    fig.PaperPositionMode = 'auto';
    fig_pos = fig.PaperPosition;
    fig.PaperSize = [fig_x_scale*fig_pos(3) fig_y_scale*fig_pos(4)];
    set(gcf, 'name', 'bocv', 'NumberTitle', 'off');
    set(gcf,'color','w');
    ylabels = ["v_x", "v_y", "v_z"];
    for i=1:3
        subplot(3, 1, i); hold on; grid on;
        ylabel(ylabels(i));
        plot(command_bo(1,1:skip:end), command_bo(i+7,1:skip:end), 'r', 'linewidth', 1.25);
        plot(target_truth_bo(1,1:skip:end), vel_true_bo(i,1:skip:end), 'b', 'linewidth', 1.5);
        if i == 1
            title('Bearings-only Commanded and True Velocities')
            legend('command', 'truth','location','southeast')
        end
        if i == 3
            xlabel('Time (seconds)')
        end
    end
    print(gcf, '~/Dropbox/dev/acc_2019/figures/bo_cmd_vel','-dpdf')
    
else
    
    % Relative position and velocity
    fig = figure('Renderer', 'painters', 'Position', [10 10 600 600]);
    fig.PaperPositionMode = 'auto';
    fig_pos = fig.PaperPosition;
    fig.PaperSize = [0.88*fig_pos(3) 0.91*fig_pos(4)];
    set(gcf, 'name', 'rbpv', 'NumberTitle', 'off');
    set(gcf,'color','w');
    ylabels = ["p_x", "p_y", "p_z", "v_x", "v_y", "v_z"];
    for i=1:6
        subplot(6, 1, i); hold on; grid on;
        ylabel(ylabels(i));
        plot(target_truth_rb(1,1:skip:end), target_truth_rb(i+1,1:skip:end), 'b', 'linewidth', 1.5);
        plot(target_est_rb(1,1:skip:end), target_est_rb(i+1,1:skip:end), 'r', 'linewidth', 1.25);
        if i == 1
            title('Range+Bearings Relative Target Position and Velocity')
            legend('truth', 'estimate','location','southeast')
        end
        if i == 6
            xlabel('Time (seconds)')
        end
    end
    print(gcf, '~/Dropbox/dev/acc_2019/figures/rb_pos_vel','-dpdf')
    
    % Relative radius and altitude
    fig = figure('Renderer', 'painters', 'Position', [10 10 fig_width fig_width*2/3]);
    fig.PaperPositionMode = 'auto';
    fig_pos = fig.PaperPosition;
    fig.PaperSize = [fig_x_scale*fig_pos(3) fig_y_scale*fig_pos(4)];
    set(gcf, 'name', 'rbrh', 'NumberTitle', 'off');
    set(gcf,'color','w');
    ylabels = ["r", "h"];
    for i=1:2
        subplot(2, 1, i); hold on; grid on;
        ylabel(ylabels(i));
        plot(target_truth_rb(1,1:skip:end), rh_true_rb(i,1:skip:end), 'b', 'linewidth', 1.5);
        plot(target_est_rb(1,1:skip:end), rh_est_rb(i,1:skip:end), 'r', 'linewidth', 1.25);
        if i == 1
            title('Range+Bearings Relative Target Radius and Altitude')
            legend('truth', 'estimate','location','southeast')
        end
        if i == 2
            xlabel('Time (seconds)')
        end
    end
    print(gcf, '~/Dropbox/dev/acc_2019/figures/rb_rh','-dpdf')
    
    % Commanded and true velocity
    fig = figure('Renderer', 'painters', 'Position', [10 10 fig_width fig_width*2/3]);
    fig.PaperPositionMode = 'auto';
    fig_pos = fig.PaperPosition;
    fig.PaperSize = [fig_x_scale*fig_pos(3) fig_y_scale*fig_pos(4)];
    set(gcf, 'name', 'rbcv', 'NumberTitle', 'off');
    set(gcf,'color','w');
    ylabels = ["v_x", "v_y", "v_z"];
    for i=1:3
        subplot(3, 1, i); hold on; grid on;
        ylabel(ylabels(i));
        plot(command_rb(1,1:skip:end), command_rb(i+7,1:skip:end), 'r', 'linewidth', 1.25);
        plot(target_truth_rb(1,1:skip:end), vel_true_rb(i,1:skip:end), 'b', 'linewidth', 1.5);
        if i == 1
            title('Range+Bearings Commanded and True Velocities')
            legend('command', 'truth','location','southeast')
        end
        if i == 3
            xlabel('Time (seconds)')
        end
    end
    print(gcf, '~/Dropbox/dev/acc_2019/figures/rb_cmd_vel','-dpdf')
    
end
