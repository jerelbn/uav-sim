function plot_quad_ekf(params, plot_cov, plot_2d_pix)

% Load data
true_state = reshape(fread(fopen(strcat(['/tmp/',params.name,'_ekf_truth.log']), 'r'), 'double'), 1 + 15 + 3 * params.ekf_num_features, []); % [time;pos;vel;euler;acc_bias;gyro_bias;pix;rho]
ekf_state = reshape(fread(fopen(strcat(['/tmp/',params.name,'_ekf_est.log']), 'r'), 'double'), 1 + 15 + 3 * params.ekf_num_features, []); % [time;pos;vel;euler;acc_bias;gyro_bias;pix;rho]
ekf_cov = reshape(fread(fopen(strcat(['/tmp/',params.name,'_ekf_cov.log']), 'r'), 'double'), 1 + 15 + 3 * params.ekf_num_features, []); % [time;pos;vel;att;acc_bias;gyro_bias;pix;rho]


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


idx = 16;
for j = 1:params.ekf_num_features
    figure()
    set(gcf, 'name', strcat(['EKF Pixel/Depth ',int2str(j)]), 'NumberTitle', 'off');
    titles = ["pix_x","pix_y","depth"];
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
            ylim([0,params.image_size{1}])
            legend('Truth', 'EKF')
        end
        if i == 2
            ylim([0,params.image_size{2}])
        end
    end
    idx = idx + 3;
end


if plot_2d_pix
    idx = 16;
    for j = 1:params.ekf_num_features
        figure(), hold on, grid on
        set(gcf, 'name', strcat(['EKF 2D Pixel/Depth ',int2str(j)]), 'NumberTitle', 'off');
        set(gca, 'YDir', 'Reverse')
        title("2D Pixel Position")
        plot(true_state(idx+1, :), true_state(idx+2, :), 'linewidth', 2.0)
        plot(ekf_state(idx+1, :), ekf_state(idx+2, :), 'linewidth', 1.5)
        legend('Truth', 'EKF')
        xlim([0,params.image_size{1}])
        ylim([0,params.image_size{2}])
        idx = idx + 3;
    end
end

end % function plot_quad_ekf
