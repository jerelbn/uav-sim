%% Plot the accelerometer measurements
figure(f); clf; f=f+1;
set(gcf, 'name', 'Accelerometer', 'NumberTitle', 'off');
titles = ["ax", "ay", "az"];
idx = 1;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(accel(1,:), accel(i + idx, :) - accel_bias(i,:) - accel_noise(i,:), 'linewidth', 1.3);
    plot(accel(1,:), accel(i + idx, :), 'r');
end

%% Plot the rate gyro measurements
figure(f); clf; f=f+1;
set(gcf, 'name', 'Rate Gyro', 'NumberTitle', 'off');
titles = ["wx", "wy", "wz"];
idx = 1;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(gyro(1,:), gyro(i + idx, :) - gyro_bias(i,:) - gyro_noise(i,:), 'linewidth', 1.3);
    plot(gyro(1,:), gyro(i + idx, :), 'r');
end
