%% Plot the rate gyro measurements
figure(f); clf; f=f+1;
set(gcf, 'name', 'Gyro Measurements', 'NumberTitle', 'off');
titles = ["x","y","z"];
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(gyro(1,:), gyro(i+1,:) - gyro(i+4,:) - gyro(i+7,:), 'linewidth', 1.3);
    plot(gyro(1,:), gyro(i+1,:), 'r');
    legend('measurement', 'truth')
end

%% Plot the accelerometer measurements
figure(f); clf; f=f+1;
set(gcf, 'name', 'Accelerometer Measurements', 'NumberTitle', 'off');
titles = ["x","y","z"];
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(accel(1,:), accel(i+1,:) - accel(i+4,:) - accel(i+7,:), 'linewidth', 1.3);
    plot(accel(1,:), accel(i+1,:), 'r');
    legend('measurement', 'truth')
end