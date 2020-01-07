function plot_gimbal_ekf(params, plot_cov, plot_pix_components, plot_2d_pix)

% Load data
true_state = reshape(fread(fopen(strcat(['/tmp/',params.name,'_ekf_truth.log']), 'r'), 'double'), 1 + 11, []); % [time;vel;euler;acc_scale;gyro_bias;mag_bias]
ekf_state = reshape(fread(fopen(strcat(['/tmp/',params.name,'_ekf_est.log']), 'r'), 'double'), 1 + 11, []); % [time;vel;euler;acc_scale;gyro_bias;mag_bias]
ekf_cov = reshape(fread(fopen(strcat(['/tmp/',params.name,'_ekf_cov.log']), 'r'), 'double'), 1 + 11, []); % [time;vel;att;acc_scale;gyro_bias;mag_bias]


figure()
set(gcf, 'name', 'EKF Velocity', 'NumberTitle', 'off')
titles = ["North", "East", "Down"];
idx = 1;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 2.0)
    plot(ekf_state(1,:), ekf_state(i + idx, :), 'r-', 'linewidth', 1.5)
    if plot_cov == true
        plot(ekf_state(1,:), ekf_state(i + idx, :) + 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
        plot(ekf_state(1,:), ekf_state(i + idx, :) - 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
    end
    if i == 1
        legend('Truth', 'EKF')
    end
    if i == 3
        set(gca, 'YDir', 'Reverse')
    end
end


figure()
set(gcf, 'name', 'EKF Attitude', 'NumberTitle', 'off')
titles = ["Roll (mrad)","Pitch (mrad)","Yaw (mrad)"];
idx = 4;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :)*1000, 'linewidth', 2.0)
    plot(ekf_state(1,:), ekf_state(i + idx, :)*1000, 'linewidth', 1.5)
    if plot_cov == true
        plot(ekf_state(1,:), ekf_state(i + idx, :)*1000 + 2 * sqrt(ekf_cov(i + idx, :))*1000, 'm-', 'linewidth', 0.5)
        plot(ekf_state(1,:), ekf_state(i + idx, :)*1000 - 2 * sqrt(ekf_cov(i + idx, :))*1000, 'm-', 'linewidth', 0.5)
    end
    if i == 1
        legend('Truth', 'EKF')
    end
end


figure(), hold on, grid on
set(gcf, 'name', 'EKF Accel Scale', 'NumberTitle', 'off')
idx = 7;
title("Accel Scale")
plot(true_state(1,:), true_state(1 + idx, :), 'linewidth', 2.0)
plot(ekf_state(1,:), ekf_state(1 + idx, :), 'linewidth', 1.5)
if plot_cov == true
    plot(ekf_state(1,:), ekf_state(1 + idx, :) + 2 * sqrt(ekf_cov(1 + idx, :)), 'm-', 'linewidth', 0.5)
    plot(ekf_state(1,:), ekf_state(1 + idx, :) - 2 * sqrt(ekf_cov(1 + idx, :)), 'm-', 'linewidth', 0.5)
end
legend('Truth', 'EKF')


figure()
set(gcf, 'name', 'EKF Gyro Bias', 'NumberTitle', 'off')
titles = ["x (mrad)","y (mrad)","z (mrad)"];
idx = 8;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :)*1000, 'linewidth', 2.0)
    plot(ekf_state(1,:), ekf_state(i + idx, :)*1000, 'linewidth', 1.5)
    if plot_cov == true
        plot(ekf_state(1,:), ekf_state(i + idx, :)*1000 + 2 * sqrt(ekf_cov(i + idx, :))*1000, 'm-', 'linewidth', 0.5)
        plot(ekf_state(1,:), ekf_state(i + idx, :)*1000 - 2 * sqrt(ekf_cov(i + idx, :))*1000, 'm-', 'linewidth', 0.5)
    end
    if i == 1
        legend('Truth', 'EKF')
    end
end


figure(), hold on, grid on
set(gcf, 'name', 'EKF Mag Bias', 'NumberTitle', 'off')
idx = 11;
title("Mag Bias (mrad)")
plot(true_state(1,:), true_state(1 + idx, :)*1000, 'linewidth', 2.0)
plot(ekf_state(1,:), ekf_state(1 + idx, :)*1000, 'linewidth', 1.5)
if plot_cov == true
    plot(ekf_state(1,:), ekf_state(1 + idx, :)*1000 + 2 * sqrt(ekf_cov(1 + idx, :))*1000, 'm-', 'linewidth', 0.5)
    plot(ekf_state(1,:), ekf_state(1 + idx, :)*1000 - 2 * sqrt(ekf_cov(1 + idx, :))*1000, 'm-', 'linewidth', 0.5)
end
legend('Truth', 'EKF')


% if plot_pix_components
%     idx = nbd+1;
%     for j = 1:params.num_features
%         figure()
%         set(gcf, 'name', strcat(['EKF Pixel/Depth ',int2str(j)]), 'NumberTitle', 'off');
%         titles = ["pix_x","pix_y","depth"];
%         for i=1:3
%             subplot(3, 1, i), hold on, grid on
%             title(titles(i))
%             plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 2.0)
%             plot(ekf_state(1,:), ekf_state(i + idx, :), 'linewidth', 1.5)
%             if plot_cov == true
%                 plot(ekf_state(1,:), ekf_state(i + idx, :) + 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
%                 plot(ekf_state(1,:), ekf_state(i + idx, :) - 2 * sqrt(ekf_cov(i + idx, :)), 'm-', 'linewidth', 0.5)
%             end
%             if i == 1
%                 ylim([0,params.image_size{1}])
%                 legend('Truth', 'EKF')
%             end
%             if i == 2
%                 ylim([0,params.image_size{2}])
%             end
%         end
%         idx = idx + 3;
%     end
% end
% 
% 
% if plot_2d_pix
%     idx = 16;
%     for j = 1:params.num_features
%         figure(), hold on, grid on
%         set(gcf, 'name', strcat(['EKF 2D Pixel/Depth ',int2str(j)]), 'NumberTitle', 'off');
%         set(gca, 'YDir', 'Reverse')
%         title("2D Pixel Position")
%         plot(true_state(idx+1, :), true_state(idx+2, :), 'linewidth', 2.0)
%         plot(ekf_state(idx+1, :), ekf_state(idx+2, :), 'linewidth', 1.5)
%         legend('Truth', 'EKF')
%         xlim([0,params.image_size{1}])
%         ylim([0,params.image_size{2}])
%         idx = idx + 3;
%     end
% end
