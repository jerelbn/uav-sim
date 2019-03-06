function plot_vehicle(name)

cam_max_feat = 10000;

% Load data
true_state = reshape(fread(fopen(strcat(['/tmp/',name,'_true_state.log']), 'r'), 'double'), 1 + 19, []);
commanded_state = reshape(fread(fopen(strcat(['/tmp/',name,'_commanded_state.log']), 'r'), 'double'), 1 + 19, []);
command = reshape(fread(fopen(strcat(['/tmp/',name,'_command.log']), 'r'), 'double'), 1 + 4, []);
euler_angles = reshape(fread(fopen(strcat(['/tmp/',name,'_euler_angles.log']), 'r'), 'double'), 1 + 3, []);
euler_command = reshape(fread(fopen(strcat(['/tmp/',name,'_euler_command.log']), 'r'), 'double'), 1 + 3, []);
accel = reshape(fread(fopen(strcat(['/tmp/',name,'_accel.log']), 'r'), 'double'), 10, []);
accel_bias = accel(5:7,:);
accel_noise = accel(8:10,:);
gyro = reshape(fread(fopen(strcat(['/tmp/',name,'_gyro.log']), 'r'), 'double'), 10, []);
gyro_bias = gyro(5:7,:);
gyro_noise = gyro(8:10,:);
baro = reshape(fread(fopen(strcat(['/tmp/',name,'_baro.log']), 'r'), 'double'), 4, []);
mocap = reshape(fread(fopen(strcat(['/tmp/',name,'_mocap.log']), 'r'), 'double'), 21, []);
pix = reshape(fread(fopen(strcat(['/tmp/',name,'_camera.log']), 'r'), 'double'), 3*cam_max_feat+1, []);


figure()
set(gcf, 'name', 'Position', 'NumberTitle', 'off');
titles = ["pn", "pe", "pd"];
idx = 1;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 1.3)
    plot(commanded_state(1,:), commanded_state(i + idx, :), 'g--')
    legend('truth', 'control')
end


figure()
set(gcf, 'name', 'Velocity', 'NumberTitle', 'off');
titles = ['u','v','w'];
idx1 = 4;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx1, :), 'linewidth', 1.3)
    plot(commanded_state(1,:), commanded_state(i + idx1, :), 'g--')
end


figure()
set(gcf, 'name', 'Attitude', 'NumberTitle', 'off');
titles = ["w","x","y","z"];
idx1 = 10;
for i=1:4
    subplot(4, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx1, :), 'linewidth', 1.3)
    plot(commanded_state(1,:), commanded_state(i + idx1, :), 'g--')
end


figure()
set(gcf, 'name', 'Euler Angles', 'NumberTitle', 'off');
titles = ["Roll","Pitch","Yaw"];
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(euler_angles(1,:), euler_angles(i + 1, :), 'linewidth', 1.3)
    plot(euler_command(1,:), euler_command(i + 1, :), 'g--')
end


figure()
set(gcf, 'name', 'Angular Rate', 'NumberTitle', 'off');
titles = ['p','q','r'];
idx1 = 14;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx1, :), 'linewidth', 1.3)
    plot(commanded_state(1,:), commanded_state(i + idx1, :), 'g--')
end


figure()
set(gcf, 'name', 'Gyro Bias', 'NumberTitle', 'off');
titles = ["x","y","z"];
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(gyro(1,:), gyro_bias(i,:), 'linewidth', 1.3)
end


figure()
set(gcf, 'name', 'Accelerometer Bias', 'NumberTitle', 'off');
titles = ["x","y","z"];
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(accel(1,:), accel_bias(i,:), 'linewidth', 1.3)
end


figure()
set(gcf, 'name', 'Barometer Bias', 'NumberTitle', 'off');
title("Barometer Bias")
grid on
plot(baro(1,:), baro(3,:), 'linewidth', 1.3)


figure()
set(gcf, 'name', 'Commands', 'NumberTitle', 'off');
if name(1:4) == "wing"
    titles = ["Aileron","Elevator","Throttle","Rudder"];
else
    titles = ["Throttle","Rate X","Rate Y","Rate Z"];
end
for i=1:4
    subplot(4, 1, i), hold on, grid on
    title(titles(i))
    plot(command(1,:), command(i + 1, :), 'linewidth', 1.3)
end
