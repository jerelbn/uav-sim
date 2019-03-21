function plot_ekf(name, plot_cov)

% Load data
true_state = reshape(fread(fopen(strcat(['/tmp/',name,'_ekf_truth.log']), 'r'), 'double'), 1 + 18, []); % [time;pos;vel;euler;acc_bias;gyro_bias;pix;rho]
ekf_state = reshape(fread(fopen(strcat(['/tmp/',name,'_ekf_est.log']), 'r'), 'double'), 1 + 18, []); % [time;pos;vel;euler;acc_bias;gyro_bias;pix;rho]
ekf_cov = reshape(fread(fopen(strcat(['/tmp/',name,'_ekf_cov.log']), 'r'), 'double'), 1 + 18, []); % [time;pos;vel;att;acc_bias;gyro_bias;pix;rho]


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
            plot(ekf_state(1,:), ekf_state(i + idx, :) + 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
            plot(ekf_state(1,:), ekf_state(i + idx, :) - 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
        else
            plot(ekf_state(1,:), -ekf_state(i + idx, :) + 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
            plot(ekf_state(1,:), -ekf_state(i + idx, :) - 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
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
            plot(ekf_state(1,:), ekf_state(i + idx, :) + 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
            plot(ekf_state(1,:), ekf_state(i + idx, :) - 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
        else
            plot(ekf_state(1,:), -ekf_state(i + idx, :) + 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
            plot(ekf_state(1,:), -ekf_state(i + idx, :) - 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
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
        plot(ekf_state(1,:), ekf_state(i + idx, :) + 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
        plot(ekf_state(1,:), ekf_state(i + idx, :) - 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
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
        plot(ekf_state(1,:), ekf_state(i + idx, :) + 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
        plot(ekf_state(1,:), ekf_state(i + idx, :) - 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
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
        plot(ekf_state(1,:), ekf_state(i + idx, :) + 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
        plot(ekf_state(1,:), ekf_state(i + idx, :) - 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
    end
    if i == 1
        legend('Truth', 'EKF')
    end
end


figure()
set(gcf, 'name', 'EKF Pixel/Depth', 'NumberTitle', 'off');
titles = ["pix_x","pix_y","depth"];
idx = 16;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 2.0)
    plot(ekf_state(1,:), ekf_state(i + idx, :), 'linewidth', 1.5)
    if plot_cov == true
        plot(ekf_state(1,:), ekf_state(i + idx, :) + 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
        plot(ekf_state(1,:), ekf_state(i + idx, :) - 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
    end
    if i == 1
        legend('Truth', 'EKF')
    end
end


figure(), hold on, grid on
set(gcf, 'name', 'EKF 2D Pixel/Depth', 'NumberTitle', 'off');
set(gca, 'YDir', 'Reverse')
title("2D Pixel Position")
plot(true_state(17, :), true_state(18, :), 'linewidth', 2.0)
plot(ekf_state(17, :), ekf_state(18, :), 'linewidth', 1.5)
legend('Truth', 'EKF')

end
