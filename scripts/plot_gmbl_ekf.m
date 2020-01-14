% Prepare Variables for Plotting
clear, close all
format compact
set(0,'DefaultFigureWindowStyle','docked')
addpath('../lib/yamlmatlab')

% Plotting variables
plot_cov = true;

% Load data
gimbal_params = ReadYaml('../params/gimbal1.yaml');

true_state = reshape(fread(fopen(gimbal_params.logname_ekf_truth, 'r'), 'double'), 1 + 11, []); % [time;vel;euler;acc_scale;gyro_bias;mag_bias]
ekf_state = reshape(fread(fopen(gimbal_params.logname_ekf_est, 'r'), 'double'), 1 + 11, []); % [time;vel;euler;acc_scale;gyro_bias;mag_bias]
ekf_cov = reshape(fread(fopen(gimbal_params.logname_ekf_cov, 'r'), 'double'), 1 + 11, []); % [time;vel;att;acc_scale;gyro_bias;mag_bias]


figure()
set(gcf, 'name', 'EKF Velocity', 'NumberTitle', 'off')
titles = ["Velocity North (m/s)", "Velocity East (m/s)", "Velocity Down (m/s)"];
idx = 1;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'b-', 'linewidth', 2.0, 'DisplayName', 'Truth')
    plot(ekf_state(1,:), ekf_state(i + idx, :), 'r-', 'linewidth', 1.5, 'DisplayName', 'EKF')
    if plot_cov == true
        plot(ekf_state(1,:), ekf_state(i + idx, :) + 2 * sqrt(ekf_cov(i + idx, :)), 'r:', 'linewidth', 0.5, 'DisplayName', '2\sigma Bound')
        plot(ekf_state(1,:), ekf_state(i + idx, :) - 2 * sqrt(ekf_cov(i + idx, :)), 'r:', 'linewidth', 0.5, 'HandleVisibility', 'off')
    end
    if i == 1
        legend()
    end
    if i == 3
        xlabel('Time (seconds)')
        set(gca, 'YDir', 'Reverse')
    end
end


figure()
set(gcf, 'name', 'EKF Attitude', 'NumberTitle', 'off')
titles = ["Attitude Roll (mrad)","Attitude Pitch (mrad)","Attitude Yaw (mrad)"];
idx = 4;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :)*1000, 'b-', 'linewidth', 2.0, 'DisplayName', 'Truth')
    plot(ekf_state(1,:), ekf_state(i + idx, :)*1000, 'r-', 'linewidth', 1.5, 'DisplayName', 'EKF')
    if plot_cov == true
        plot(ekf_state(1,:), ekf_state(i + idx, :)*1000 + 2 * sqrt(ekf_cov(i + idx, :))*1000, 'r:', 'linewidth', 0.5, 'DisplayName', '2\sigma Bound')
        plot(ekf_state(1,:), ekf_state(i + idx, :)*1000 - 2 * sqrt(ekf_cov(i + idx, :))*1000, 'r:', 'linewidth', 0.5, 'HandleVisibility', 'off')
    end
    if i == 1
        legend()
    end
    if i == 3
        xlabel('Time (seconds)')
    end
end


figure()
set(gcf, 'name', 'EKF Gyro Bias', 'NumberTitle', 'off')
titles = ["Gyro Bias X (mrad)","Gyro Bias Y (mrad)","Gyro Bias Z (mrad)"];
idx = 8;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :)*1000, 'b-', 'linewidth', 2.0, 'DisplayName', 'Truth')
    plot(ekf_state(1,:), ekf_state(i + idx, :)*1000, 'r-', 'linewidth', 1.5, 'Displayname', 'EKF')
    if plot_cov == true
        plot(ekf_state(1,:), ekf_state(i + idx, :)*1000 + 2 * sqrt(ekf_cov(i + idx, :))*1000, 'r:', 'linewidth', 0.5, 'DisplayName', '2\sigma Bound')
        plot(ekf_state(1,:), ekf_state(i + idx, :)*1000 - 2 * sqrt(ekf_cov(i + idx, :))*1000, 'r:', 'linewidth', 0.5, 'HandleVisibility', 'off')
    end
    if i == 1
        legend()
    end
    if i == 3
        xlabel('Time (seconds)')
    end
end


figure()
set(gcf, 'name', 'EKF Accel Scale and Mag Bias', 'NumberTitle', 'off')
subplot(2, 1, 1), hold on, grid on
title("Accel Scale")
plot(true_state(1,:), true_state(1 + 7, :), 'b-', 'linewidth', 2.0, 'DisplayName', 'Truth')
plot(ekf_state(1,:), ekf_state(1 + 7, :), 'r-', 'linewidth', 1.5, 'DisplayName', 'EKF')
if plot_cov == true
    plot(ekf_state(1,:), ekf_state(1 + 7, :) + 2 * sqrt(ekf_cov(1 + 7, :)), 'r:', 'linewidth', 0.5, 'DisplayName', '2\sigma Bound')
    plot(ekf_state(1,:), ekf_state(1 + 7, :) - 2 * sqrt(ekf_cov(1 + 7, :)), 'r:', 'linewidth', 0.5, 'HandleVisibility', 'off')
end
legend()
subplot(2, 1, 2), hold on, grid on
title("Mag Bias (mrad)")
plot(true_state(1,:), true_state(1 + 11, :)*1000, 'b-', 'linewidth', 2.0, 'DisplayName', 'Truth')
plot(ekf_state(1,:), ekf_state(1 + 11, :)*1000, 'r-', 'linewidth', 1.5, 'DisplayName', 'EKF')
if plot_cov == true
    plot(ekf_state(1,:), ekf_state(1 + 11, :)*1000 + 2 * sqrt(ekf_cov(1 + 11, :))*1000, 'r:', 'linewidth', 0.5, 'DisplayName', '2\sigma Bound')
    plot(ekf_state(1,:), ekf_state(1 + 11, :)*1000 - 2 * sqrt(ekf_cov(1 + 11, :))*1000, 'r:', 'linewidth', 0.5, 'HandleVisibility', 'off')
end
xlabel('Time (seconds)')
