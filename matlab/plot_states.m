%% Plot the position states
cov_color = 0.5;
sigma = 2;

figure(f); clf; f=f+1;
set(gcf, 'name', 'Position', 'NumberTitle', 'off');
titles = ["pn", "pe", "pd"];
idx = 1;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 1.3);
    plot(true_state(1,:), command(i + idx, :), 'g--');
    legend('truth', 'control')
end

%% plot the attitude states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Attitude', 'NumberTitle', 'off');
titles = ["w","x","y","z"];
idx1 = 10;
for i=1:4
    subplot(4, 1, i); hold on;
    title(titles(i));
    plot(true_state(1,:), true_state(i + idx1, :), 'linewidth', 1.3);
    legend('truth')
end

%% Plot the velocity states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Velocity', 'NumberTitle', 'off');
titles = ['u','v','w'];
idx1 = 4;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(true_state(1,:), true_state(i + idx1, :), 'linewidth', 1.3);
    legend('truth')
end

%% Plot the gyro bias
figure(f); clf; f=f+1;
set(gcf, 'name', 'Gyro Bias', 'NumberTitle', 'off');
titles = ["x","y","z"];
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(gyro(1,:), gyro_bias(i,:), 'linewidth', 1.3);
    legend('truth')
end

%% Plot the accel bias
figure(f); clf; f=f+1;
set(gcf, 'name', 'Accelerometer Bias', 'NumberTitle', 'off');
titles = ["x","y","z"];
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(accel(1,:), accel_bias(i,:), 'linewidth', 1.3);
    legend('truth')
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
    legend('truth')
end
