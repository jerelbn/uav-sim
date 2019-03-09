function plot_vehicle(name, plot_cov)

cam_max_feat = 10000;

% Load data
true_state = reshape(fread(fopen(strcat(['/tmp/',name,'_true_state.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;accel;att;ang_vel;ang_accel]
ekf_state = reshape(fread(fopen(strcat(['/tmp/',name,'_ekf_state.log']), 'r'), 'double'), 1 + 20, []); % [time;pos;vel;att;acc_bias;gyro_bias;wind_inertial;baro_bias]
ekf_euler = reshape(fread(fopen(strcat(['/tmp/',name,'_ekf_euler.log']), 'r'), 'double'), 1 + 3, []); % [time;roll;pitch;yaw]
ekf_cov = reshape(fread(fopen(strcat(['/tmp/',name,'_ekf_cov.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;att;acc_bias;gyro_bias;wind_inertial;baro_bias]
commanded_state = reshape(fread(fopen(strcat(['/tmp/',name,'_commanded_state.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;accel;att;ang_vel;ang_accel]
command = reshape(fread(fopen(strcat(['/tmp/',name,'_command.log']), 'r'), 'double'), 1 + 4, []); % [time;aileron;elevator;throttle;rudder]
euler_angles = reshape(fread(fopen(strcat(['/tmp/',name,'_euler_angles.log']), 'r'), 'double'), 1 + 3, []); % [time;roll;pitch;yaw]
euler_command = reshape(fread(fopen(strcat(['/tmp/',name,'_euler_command.log']), 'r'), 'double'), 1 + 3, []); % [time;roll;pitch;yaw]
accel = reshape(fread(fopen(strcat(['/tmp/',name,'_accel.log']), 'r'), 'double'), 10, []); % [time;accel;bias;noise]
gyro = reshape(fread(fopen(strcat(['/tmp/',name,'_gyro.log']), 'r'), 'double'), 10, []); % [time;gyro;bias;noise]
mocap = reshape(fread(fopen(strcat(['/tmp/',name,'_mocap.log']), 'r'), 'double'), 21, []); % [time;pos;att;pos_body2mocap;att_body2mocap;pos_noise;att_noise]
baro = reshape(fread(fopen(strcat(['/tmp/',name,'_baro.log']), 'r'), 'double'), 4, []); % [time;baro;bias;noise]
pitot = reshape(fread(fopen(strcat(['/tmp/',name,'_pitot.log']), 'r'), 'double'), 4, []); % [time;pitot;bias;noise]
wvane = reshape(fread(fopen(strcat(['/tmp/',name,'_wvane.log']), 'r'), 'double'), 3, []); % [time;wvane_meas;wvane_true]
gps = reshape(fread(fopen(strcat(['/tmp/',name,'_gps.log']), 'r'), 'double'), 19, []); % [time;pos;vel;pos_bias;vel_bias;pos_noise;vel_noise]
pix = reshape(fread(fopen(strcat(['/tmp/',name,'_camera.log']), 'r'), 'double'), 3*cam_max_feat+1, []); % [pix_x;pix_y;pix_id]


figure()
set(gcf, 'name', 'Position', 'NumberTitle', 'off');
titles = ["North", "East", "Altitude"];
idx = 1;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 3)
    plot(ekf_state(1,:), ekf_state(i + idx, :), 'r-', 'linewidth', 1.5)
    plot(commanded_state(1,:), commanded_state(i + idx, :), 'g--')
    if plot_cov == true
        plot(ekf_state(1,:), ekf_state(i + idx, :) + sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
        plot(ekf_state(1,:), ekf_state(i + idx, :) - sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
    end
    if i == 1
        legend('True', 'EKF', 'Command')
    end
end


figure()
set(gcf, 'name', 'Velocity', 'NumberTitle', 'off');
titles = ['u','v','w'];
idx = 4;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 3)
    plot(ekf_state(1,:), ekf_state(i + idx, :), 'r-', 'linewidth', 1.5)
    plot(commanded_state(1,:), commanded_state(i + idx, :), 'g--')
    if plot_cov == true
        plot(ekf_state(1,:), ekf_state(i + idx, :) + sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
        plot(ekf_state(1,:), ekf_state(i + idx, :) - sqrt(ekf_cov(i + idx, :)), 'r-', 'linewidth', 0.5)
    end
    if i == 1
        legend('True', 'EKF', 'Command')
    end
end


figure()
set(gcf, 'name', 'Attitude', 'NumberTitle', 'off');
titles = ["w","x","y","z"];
idx = 10;
for i=1:4
    subplot(4, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 1.5)
    plot(commanded_state(1,:), commanded_state(i + idx, :), 'g--')
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
    plot(euler_angles(1,:), euler_angles(i + 1, :), 'linewidth', 3.0)
    plot(ekf_euler(1,:), ekf_euler(i + 1, :), 'linewidth', 1.5)
    if plot_cov == true
        plot(ekf_euler(1,:), ekf_euler(i + 1, :) + sqrt(ekf_cov(i + 7, :)), 'r-', 'linewidth', 0.5)
        plot(ekf_euler(1,:), ekf_euler(i + 1, :) - sqrt(ekf_cov(i + 7, :)), 'r-', 'linewidth', 0.5)
    end
    plot(euler_command(1,:), euler_command(i + 1, :), 'g--')
    if i == 1
        legend('True', 'EKF', 'Command')
    end
end


figure()
set(gcf, 'name', 'Angular Rate', 'NumberTitle', 'off');
titles = ['p','q','r'];
idx = 14;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 1.5)
    plot(commanded_state(1,:), commanded_state(i + idx, :), 'g--')
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
    plot(accel(1,:), accel(i+1,:)-accel(i+4,:)-accel(i+7,:), 'b-', 'linewidth', 1.5)
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
    plot(gyro(1,:), gyro(i+1,:)-gyro(i+4,:)-gyro(i+7,:), 'b-', 'linewidth', 1.5)
    if i == 1
        legend('Measured', 'True')
    end
end


figure()
set(gcf, 'name', 'Barometer', 'NumberTitle', 'off');
title("Barometer")
grid on, hold on
plot(baro(1,:), baro(2,:), 'r-', 'linewidth', 1.5)
plot(baro(1,:), baro(2,:)-baro(3,:)-baro(4,:), 'b-', 'linewidth', 1.5)
legend('Measured', 'True')


figure()
set(gcf, 'name', 'Pitot Tube', 'NumberTitle', 'off');
title("Pitot Tube")
grid on, hold on
plot(pitot(1,:), pitot(2,:), 'r-', 'linewidth', 1.5)
plot(pitot(1,:), pitot(2,:)-pitot(3,:)-pitot(4,:), 'b-', 'linewidth', 1.5)
legend('Measured', 'True')


figure()
set(gcf, 'name', 'Weather Vane', 'NumberTitle', 'off');
title("Weather Vane")
grid on, hold on
plot(wvane(1,:), wvane(2,:), 'r-', 'linewidth', 1.5)
plot(wvane(1,:), wvane(3,:), 'b-', 'linewidth', 1.5)
legend('Measured', 'True')


figure()
set(gcf, 'name', 'GPS Position', 'NumberTitle', 'off');
titles = ["North","East","Altitude"];
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    if i < 3
        plot(gps(1,:), gps(i+1,:), 'r-', 'linewidth', 1.5)
        plot(gps(1,:), gps(i+1,:)-gps(i+7,:)-gps(i+13,:), 'b-', 'linewidth', 1.5)
    else
        plot(gps(1,:), -gps(i+1,:), 'r-', 'linewidth', 1.5)
        plot(gps(1,:), -gps(i+1,:)+gps(i+7,:)+gps(i+13,:), 'b-', 'linewidth', 1.5)
    end
    if i == 1
        legend('Measured', 'True')
    end
end


figure()
set(gcf, 'name', 'GPS Velocity', 'NumberTitle', 'off');
titles = ["North","East","Altitude"];
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    if i < 3
        plot(gps(1,:), gps(i+4,:), 'r-', 'linewidth', 1.5)
        plot(gps(1,:), gps(i+4,:)-gps(i+10,:)-gps(i+16,:), 'b-', 'linewidth', 1.5)
    else
        plot(gps(1,:), -gps(i+4,:), 'r-', 'linewidth', 1.5)
        plot(gps(1,:), -gps(i+4,:)+gps(i+10,:)+gps(i+16,:), 'b-', 'linewidth', 1.5)
    end
    if i == 1
        legend('Measured', 'True')
    end
end


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
    if i == 3
        ylim([0 1])
    else
        ylim([-1 1])
    end
end
