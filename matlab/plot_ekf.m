function plot_ekf(name, plot_cov)

% Load data
true_state = reshape(fread(fopen(strcat(['/tmp/',name,'_ekf_truth.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;euler;acc_bias;gyro_bias;wind_inertial;baro_bias]
ekf_state = reshape(fread(fopen(strcat(['/tmp/',name,'_ekf_est.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;euler;acc_bias;gyro_bias;wind_inertial;baro_bias]
ekf_cov = reshape(fread(fopen(strcat(['/tmp/',name,'_ekf_cov.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;att;acc_bias;gyro_bias;wind_inertial;baro_bias]


figure()
set(gcf, 'name', 'EKF Position', 'NumberTitle', 'off');
titles = ["North", "East", "Altitude"];
idx = 1;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    if i < 3
        plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 2.0)
        plot(ekf_state(1,:), ekf_state(i + idx, :), 'r-', 'linewidth', 1.5)
    else
        plot(true_state(1,:), -true_state(i + idx, :), 'linewidth', 2.0)
        plot(ekf_state(1,:), -ekf_state(i + idx, :), 'r-', 'linewidth', 1.5)
    end
    if plot_cov == true
        if i < 3
            plot(ekf_state(1,:), ekf_state(i + idx, :) + sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
            plot(ekf_state(1,:), ekf_state(i + idx, :) - sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
        else
            plot(ekf_state(1,:), -ekf_state(i + idx, :) + sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
            plot(ekf_state(1,:), -ekf_state(i + idx, :) - sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
        end
    end
    if i == 1
        legend('Truth', 'EKF')
    end
end


figure()
set(gcf, 'name', 'EKF Velocity', 'NumberTitle', 'off');
titles = ["North", "East", "Altitude"];
idx = 4;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    if i < 3
        plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 2.0)
        plot(ekf_state(1,:), ekf_state(i + idx, :), 'r-', 'linewidth', 1.5)
    else
        plot(true_state(1,:), -true_state(i + idx, :), 'linewidth', 2.0)
        plot(ekf_state(1,:), -ekf_state(i + idx, :), 'r-', 'linewidth', 1.5)
    end
    if plot_cov == true
        if i < 3
            plot(ekf_state(1,:), ekf_state(i + idx, :) + sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
            plot(ekf_state(1,:), ekf_state(i + idx, :) - sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
        else
            plot(ekf_state(1,:), -ekf_state(i + idx, :) + sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
            plot(ekf_state(1,:), -ekf_state(i + idx, :) - sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
        end
    end
    if i == 1
        legend('Truth', 'EKF')
    end
end


figure()
set(gcf, 'name', 'EKF Attitude', 'NumberTitle', 'off');
titles = ["Roll","Pitch","Yaw"];
idx = 7;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 2.0)
    plot(ekf_state(1,:), ekf_state(i + idx, :), 'linewidth', 1.5)
    if plot_cov == true
        plot(ekf_state(1,:), ekf_state(i + idx, :) + sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
        plot(ekf_state(1,:), ekf_state(i + idx, :) - sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
    end
    if i == 1
        legend('Truth', 'EKF')
    end
end


figure()
set(gcf, 'name', 'EKF Accel Bias', 'NumberTitle', 'off');
titles = ["x","y","z"];
idx = 10;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 2.0)
    plot(ekf_state(1,:), ekf_state(i + idx, :), 'linewidth', 1.5)
    if plot_cov == true
        plot(ekf_state(1,:), ekf_state(i + idx, :) + sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
        plot(ekf_state(1,:), ekf_state(i + idx, :) - sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
    end
    if i == 1
        legend('Truth', 'EKF')
    end
end


figure()
set(gcf, 'name', 'EKF Gyro Bias', 'NumberTitle', 'off');
titles = ["x","y","z"];
idx = 13;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 2.0)
    plot(ekf_state(1,:), ekf_state(i + idx, :), 'linewidth', 1.5)
    if plot_cov == true
        plot(ekf_state(1,:), ekf_state(i + idx, :) + sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
        plot(ekf_state(1,:), ekf_state(i + idx, :) - sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
    end
    if i == 1
        legend('Truth', 'EKF')
    end
end


figure()
set(gcf, 'name', 'EKF Wind', 'NumberTitle', 'off');
titles = ["x","y","z"];
idx = 16;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 2.0)
    plot(ekf_state(1,:), ekf_state(i + idx, :), 'linewidth', 1.5)
    if plot_cov == true
        plot(ekf_state(1,:), ekf_state(i + idx, :) + sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
        plot(ekf_state(1,:), ekf_state(i + idx, :) - sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
    end
    if i == 1
        legend('Truth', 'EKF')
    end
end


figure()
set(gcf, 'name', 'EKF Baro Bias', 'NumberTitle', 'off');
idx = 19;
hold on, grid on
title("Baro Bias")
plot(true_state(1,:), true_state(1 + idx, :), 'linewidth', 2.0)
plot(ekf_state(1,:), ekf_state(1 + idx, :), 'linewidth', 1.5)
if plot_cov == true
    plot(ekf_state(1,:), ekf_state(1 + idx, :) + sqrt(ekf_cov(1 + idx, :)), 'r-', 'linewidth', 0.5)
    plot(ekf_state(1,:), ekf_state(1 + idx, :) - sqrt(ekf_cov(1 + idx, :)), 'r-', 'linewidth', 0.5)
end
legend('Truth', 'EKF')
