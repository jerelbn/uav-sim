%% Plot the position states and heading
cov_color = 0.5;
sigma = 2;

figure(f); clf; f=f+1;
set(gcf, 'name', 'Position', 'NumberTitle', 'off');
titles = ["pn", "pe", "pd","heading"];
idx = 1;
for i=1:4
    subplot(4, 1, i); hold on;
    title(titles(i));
    if i < 4
        plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 1.3);
        plot(ekf_state(1,:), ekf_state(i+idx,:), 'r');
        plot(true_state(1,:), command(i + idx, :), 'g--');
        if (plot_covariance)
            plot(ekf_cov(1,:), ekf_state(i+idx,:) + sigma * sqrt(ekf_cov(i+idx,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
            plot(ekf_cov(1,:), ekf_state(i+idx,:) - sigma * sqrt(ekf_cov(i+idx,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
        end
    else
        plot(true_state(1,:), true_heading, 'linewidth', 1.3);
        plot(ekf_state(1,:), ekf_heading, 'r');
        plot(command(1,:), command(7,:), 'g--');
    end
    legend('truth', 'estimate', 'control')
end

%% plot the attitude states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Attitude', 'NumberTitle', 'off');
titles = ["w","x","y","z"];
idx1 = 10;
idx2 = 4;
for i=1:4
    subplot(4, 1, i); hold on;
    title(titles(i));
    plot(true_state(1,:), true_state(i + idx1, :), 'linewidth', 1.3);
    plot(ekf_state(1,:), ekf_state(i+idx2,:), 'r');
    if (plot_covariance && i > 1)
        plot(ekf_cov(1,:), ekf_state(i+idx2,:) + sigma * sqrt(ekf_cov(i+idx2-1,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
        plot(ekf_cov(1,:), ekf_state(i+idx2,:) - sigma * sqrt(ekf_cov(i+idx2-1,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
    end
    legend('truth', 'estimate')
end

%% Plot the velocity states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Velocity', 'NumberTitle', 'off');
titles = ['u','v','w'];
idx1 = 4;
idx2 = 8;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(true_state(1,:), true_state(i + idx1, :), 'linewidth', 1.3);
    plot(ekf_state(1,:), ekf_state(i+idx2,:), 'r');
    if (plot_covariance)
        plot(ekf_cov(1,:), ekf_state(i+idx2,:) + sigma * sqrt(ekf_cov(i+idx2-1,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
        plot(ekf_cov(1,:), ekf_state(i+idx2,:) - sigma * sqrt(ekf_cov(i+idx2-1,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
    end
    legend('truth', 'estimate')
end

%% Plot the gyro bias
figure(f); clf; f=f+1;
set(gcf, 'name', 'Gyro Bias', 'NumberTitle', 'off');
titles = ["x","y","z"];
idx = 11;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(gyro(1,:), gyro_bias(i,:), 'linewidth', 1.3);
    plot(ekf_state(1,:), ekf_state(i+idx,:), 'r');
    if (plot_covariance)
        plot(ekf_cov(1,:), ekf_state(i+idx,:) + sigma * sqrt(ekf_cov(i+idx-1,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
        plot(ekf_cov(1,:), ekf_state(i+idx,:) - sigma * sqrt(ekf_cov(i+idx-1,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
    end
    legend('truth', 'estimate')
end

%% Plot the accel bias
figure(f); clf; f=f+1;
set(gcf, 'name', 'Accelerometer Bias', 'NumberTitle', 'off');
titles = ["x","y","z"];
idx = 14;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(accel(1,:), accel_bias(i,:), 'linewidth', 1.3);
    plot(ekf_state(1,:), ekf_state(i+idx,:), 'r');
    if (plot_covariance)
        plot(ekf_cov(1,:), ekf_state(i+idx,:) + sigma * sqrt(ekf_cov(i+idx-1,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
        plot(ekf_cov(1,:), ekf_state(i+idx,:) - sigma * sqrt(ekf_cov(i+idx-1,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
    end
    legend('truth', 'estimate')
end

%% Plot the angular rates
figure(f); clf; f=f+1;
set(gcf, 'name', 'Omega', 'NumberTitle', 'off');
titles = ["x","y","z"];
idx = 14;
for i=1:3
    subplot(3,1,i); hold on;
    title(titles(i));
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 1.3);
    plot(gyro(1,:), gyro(i + 1, :), 'r');
    legend('truth', 'measured')
end

%% Plot the linear acceleration
figure(f); clf; f=f+1;
set(gcf, 'name', 'Accel', 'NumberTitle', 'off');
titles = ["x","y","z"];
idx = 7;
for i=1:3
    subplot(3,1,i); hold on;
    title(titles(i));
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 1.3);
    plot(accel(1,:), accel(i + 1, :), 'r');
    legend('truth', 'measured')
end

%% Plot the wind velocity
figure(f); clf; f=f+1;
set(gcf, 'name', 'Wind', 'NumberTitle', 'off');
titles = ["north", "east","down"];
idx = 1;
for i=1:3
    subplot(3,1,i); hold on;
    title(titles(i));
    plot(vw(1,:), vw(i + idx, :), 'linewidth', 1.3);
%     legend('truth', 'measured')
end