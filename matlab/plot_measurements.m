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

%% Plot the first camera image
figure(f); clf; f=f+1; hold on;
set(gcf, 'name', 'Camera', 'NumberTitle', 'off');
title('First Camera Image');
set(gca, 'YDir', 'reverse')
axis([0 640 0 480])
pix1 = pix(pix(:,3) >= 0);
pix1 = reshape(pix1(2:end),3,[]);
plot(pix1(1,:), pix1(2,:), 'b.');
