function plot_vehicle(name)

cam_max_feat = 10000;

% Load data
true_state = reshape(fread(fopen(strcat(['/tmp/',name,'_true_state.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;accel;att;ang_vel;ang_accel]
commanded_state = reshape(fread(fopen(strcat(['/tmp/',name,'_commanded_state.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;accel;att;ang_vel;ang_accel]
command = reshape(fread(fopen(strcat(['/tmp/',name,'_command.log']), 'r'), 'double'), 1 + 4, []); % [time;aileron;elevator;throttle;rudder]
euler_angles = reshape(fread(fopen(strcat(['/tmp/',name,'_euler_angles.log']), 'r'), 'double'), 1 + 3, []); % [time;roll;pitch;yaw]
euler_command = reshape(fread(fopen(strcat(['/tmp/',name,'_euler_command.log']), 'r'), 'double'), 1 + 3, []); % [time;roll;pitch;yaw]
accel = reshape(fread(fopen(strcat(['/tmp/',name,'_accel.log']), 'r'), 'double'), 10, []); % [time;accel;bias;noise]
gyro = reshape(fread(fopen(strcat(['/tmp/',name,'_gyro.log']), 'r'), 'double'), 10, []); % [time;gyro;bias;noise]
baro = reshape(fread(fopen(strcat(['/tmp/',name,'_baro.log']), 'r'), 'double'), 4, []); % [time;baro;bias;noise]
pitot = reshape(fread(fopen(strcat(['/tmp/',name,'_pitot.log']), 'r'), 'double'), 4, []); % [time;pitot;bias;noise]
mocap = reshape(fread(fopen(strcat(['/tmp/',name,'_mocap.log']), 'r'), 'double'), 21, []); % [time;pos;att;pos_body2mocap;att_body2mocap;pos_noise;att_noise]
pix = reshape(fread(fopen(strcat(['/tmp/',name,'_camera.log']), 'r'), 'double'), 3*cam_max_feat+1, []); % [pix_x;pix_y;pix_id]


figure()
set(gcf, 'name', 'Position', 'NumberTitle', 'off');
titles = ["North", "East", "Altitude"];
idx = 1;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    if i < 3
        plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 1.5)
        plot(commanded_state(1,:), commanded_state(i + idx, :), 'g--')
    else
        plot(true_state(1,:), -true_state(i + idx, :), 'linewidth', 1.5)
        plot(commanded_state(1,:), -commanded_state(i + idx, :), 'g--')
    end
    if i == 1
        legend('True', 'Command')
    end
end


figure()
set(gcf, 'name', 'Velocity', 'NumberTitle', 'off');
titles = ['u','v','w'];
idx1 = 4;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx1, :), 'linewidth', 1.5)
    plot(commanded_state(1,:), commanded_state(i + idx1, :), 'g--')
    if i == 1
        legend('True', 'Command')
    end
end


figure()
set(gcf, 'name', 'Attitude', 'NumberTitle', 'off');
titles = ["w","x","y","z"];
idx1 = 10;
for i=1:4
    subplot(4, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx1, :), 'linewidth', 1.5)
    plot(commanded_state(1,:), commanded_state(i + idx1, :), 'g--')
    if i == 1
        legend('True', 'Command')
    end
end


figure()
set(gcf, 'name', 'Euler Angles', 'NumberTitle', 'off');
titles = ["Roll","Pitch","Yaw"];
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(euler_angles(1,:), euler_angles(i + 1, :), 'linewidth', 1.5)
    plot(euler_command(1,:), euler_command(i + 1, :), 'g--')
    if i == 1
        legend('True', 'Command')
    end
end


figure()
set(gcf, 'name', 'Angular Rate', 'NumberTitle', 'off');
titles = ['p','q','r'];
idx1 = 14;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx1, :), 'linewidth', 1.5)
    plot(commanded_state(1,:), commanded_state(i + idx1, :), 'g--')
    if i == 1
        legend('True', 'Command')
    end
end


figure()
set(gcf, 'name', 'Accelerometer', 'NumberTitle', 'off');
titles = ["x","y","z"];
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(accel(1,:), accel(i+1,:), 'r-', 'linewidth', 1.5)
    plot(accel(1,:), accel(i+1,:)-accel(i+4,:)-accel(i+7,:), 'linewidth', 1.5)
    if i == 1
        legend('Measured', 'True')
    end
end


figure()
set(gcf, 'name', 'Gyro', 'NumberTitle', 'off');
titles = ["x","y","z"];
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(gyro(1,:), gyro(i+1,:), 'r-', 'linewidth', 1.5)
    plot(gyro(1,:), gyro(i+1,:)-gyro(i+4,:)-gyro(i+7,:), 'linewidth', 1.5)
    if i == 1
        legend('Measured', 'True')
    end
end


figure()
set(gcf, 'name', 'Barometer', 'NumberTitle', 'off');
title("Barometer")
grid on, hold on
plot(baro(1,:), baro(2,:), 'r-', 'linewidth', 1.5)
plot(baro(1,:), baro(2,:)-baro(3,:)-baro(4,:), 'linewidth', 1.5)
legend('Measured', 'True')


figure()
set(gcf, 'name', 'Pitot Tube', 'NumberTitle', 'off');
title("Pitot Tube")
grid on, hold on
plot(pitot(1,:), pitot(2,:), 'r-', 'linewidth', 1.5)
plot(pitot(1,:), pitot(2,:)-pitot(3,:)-pitot(4,:), 'linewidth', 1.5)
legend('Measured', 'True')


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
    plot(command(1,:), command(i + 1, :), 'linewidth', 1.5)
end
