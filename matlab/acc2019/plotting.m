% Prepare Variables for Plotting
clear
format compact
set(0,'DefaultFigureWindowStyle','docked')
f = 1; % Starts figure numbering

% Read binary files
directory = 'binaries/';
read_logs;

% Compute relative states for plotting
compute_relative;

% Bearings-only relative position
figure(f); clf; f=f+1;
set(gcf, 'name', 'bop', 'NumberTitle', 'off');
set(gcf,'color','w');
ylabels = ["p_x", "p_y", "p_z"];
for i=1:3
    subplot(3, 1, i); hold on; grid on;
    ylabel(ylabels(i));
    plot(target_truth_bo(1,:), target_truth_bo(i+1, :), 'b', 'linewidth', 1.5);
    plot(target_est_bo(1,:), target_est_bo(i+1,:), 'r', 'linewidth', 1.25);
    if i == 1
        title('Bearings-only Relative Target Position')
        legend('truth', 'estimate')
    end
    if i == 3
        xlabel('Time (seconds)')
    end
end

% Range+bearings relative position and velocity
figure(f); clf; f=f+1;
set(gcf, 'name', 'rbpv', 'NumberTitle', 'off');
set(gcf,'color','w');
ylabels = ["p_x", "p_y", "p_z", "v_x", "v_y", "v_z"];
for i=1:6
    subplot(6, 1, i); hold on; grid on;
    ylabel(ylabels(i));
    plot(target_truth_rb(1,:), target_truth_rb(i+1, :), 'b', 'linewidth', 1.5);
    plot(target_est_rb(1,:), target_est_rb(i+1,:), 'r', 'linewidth', 1.25);
    if i == 1
        title('Range+Bearings Relative Target Position and Velocity')
        legend('truth', 'estimate')
    end
    if i == 6
        xlabel('Time (seconds)')
    end
end

% Bearings-only relative radius and altitude
figure(f); clf; f=f+1;
set(gcf, 'name', 'borh', 'NumberTitle', 'off');
set(gcf,'color','w');
ylabels = ["r", "h"];
for i=1:2
    subplot(2, 1, i); hold on; grid on;
    ylabel(ylabels(i));
    plot(target_truth_bo(1,:), rh_true_bo(i,:), 'b', 'linewidth', 1.5);
    plot(target_est_bo(1,:), rh_est_bo(i,:), 'r', 'linewidth', 1.25);
    if i == 1
        title('Bearings-only Relative Target Radius and Altitude')
        legend('truth', 'estimate')
    end
    if i == 2
        xlabel('Time (seconds)')
    end
end

% Range+Bearings relative radius and altitude
figure(f); clf; f=f+1;
set(gcf, 'name', 'rbrh', 'NumberTitle', 'off');
set(gcf,'color','w');
ylabels = ["r", "h"];
for i=1:2
    subplot(2, 1, i); hold on; grid on;
    ylabel(ylabels(i));
    plot(target_truth_rb(1,:), rh_true_rb(i,:), 'b', 'linewidth', 1.5);
    plot(target_est_rb(1,:), rh_est_rb(i,:), 'r', 'linewidth', 1.25);
    if i == 1
        title('Range+Bearings Relative Target Radius and Altitude')
        legend('truth', 'estimate')
    end
    if i == 2
        xlabel('Time (seconds)')
    end
end

% Bearings-only commanded and true velocity
figure(f); clf; f=f+1;
set(gcf, 'name', 'bocv', 'NumberTitle', 'off');
set(gcf,'color','w');
ylabels = ["v_x", "v_y", "v_z"];
for i=1:3
    subplot(3, 1, i); hold on; grid on;
    ylabel(ylabels(i));
    plot(command_bo(1,:), command_bo(i+7,:), 'r', 'linewidth', 1.25);
    plot(target_truth_bo(1,:), vel_true_bo(i, :), 'b', 'linewidth', 1.5);
    if i == 1
        title('Bearings-only Commanded and True Velocities')
        legend('command', 'truth')
    end
    if i == 3
        xlabel('Time (seconds)')
    end
end

% Range+Bearings commanded and true velocity
figure(f); clf; f=f+1;
set(gcf, 'name', 'rbcv', 'NumberTitle', 'off');
set(gcf,'color','w');
ylabels = ["v_x", "v_y", "v_z"];
for i=1:3
    subplot(3, 1, i); hold on; grid on;
    ylabel(ylabels(i));
    plot(command_rb(1,:), command_rb(i+7,:), 'r', 'linewidth', 1.25);
    plot(target_truth_rb(1,:), vel_true_rb(i, :), 'b', 'linewidth', 1.5);
    if i == 1
        title('Range+Bearings Commanded and True Velocities')
        legend('command', 'truth')
    end
    if i == 3
        xlabel('Time (seconds)')
    end
end
