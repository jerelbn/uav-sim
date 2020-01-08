function plot_gimbal_ekf(params, plot_cov, plot_pix_components, plot_2d_pix)

% Load data
true_state = reshape(fread(fopen(strcat(['/tmp/',params.name,'_ekf_truth.log']), 'r'), 'double'), 1 + 11, []); % [time;vel;euler;acc_scale;gyro_bias;mag_bias]
ekf_state = reshape(fread(fopen(strcat(['/tmp/',params.name,'_ekf_est.log']), 'r'), 'double'), 1 + 11, []); % [time;vel;euler;acc_scale;gyro_bias;mag_bias]
ekf_cov = reshape(fread(fopen(strcat(['/tmp/',params.name,'_ekf_cov.log']), 'r'), 'double'), 1 + 11, []); % [time;vel;att;acc_scale;gyro_bias;mag_bias]


figure()
set(gcf, 'name', 'EKF Velocity', 'NumberTitle', 'off')
titles = ["Velocity North", "Velocity East", "Velocity Down"];
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
plot(true_state(1,:), true_state(1 + 7, :)*1000, 'b-', 'linewidth', 2.0, 'DisplayName', 'Truth')
plot(ekf_state(1,:), ekf_state(1 + 7, :)*1000, 'r-', 'linewidth', 1.5, 'DisplayName', 'EKF')
if plot_cov == true
    plot(ekf_state(1,:), ekf_state(1 + 7, :)*1000 + 2 * sqrt(ekf_cov(1 + 7, :))*1000, 'r:', 'linewidth', 0.5, 'DisplayName', '2\sigma Bound')
    plot(ekf_state(1,:), ekf_state(1 + 7, :)*1000 - 2 * sqrt(ekf_cov(1 + 7, :))*1000, 'r:', 'linewidth', 0.5, 'HandleVisibility', 'off')
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
