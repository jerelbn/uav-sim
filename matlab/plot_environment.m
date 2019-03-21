function plot_environment(name, params)

% Load data
env = reshape(fread(fopen(strcat('/tmp/environment.log'), 'r'), 'double'), 3, []);
true_state = reshape(fread(fopen(strcat(['/tmp/',name,'_true_state.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;accel;att;ang_vel;ang_accel]
cmd_state = reshape(fread(fopen(strcat(['/tmp/',name,'_commanded_state.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;accel;att;ang_vel;ang_accel]
bike_state = reshape(fread(fopen('/tmp/bike1_true_state.log', 'r'), 'double'), 7, []); % [pos;vel;heading;steering_angle]

% Plot 3D scene
figure(), hold on, grid on
set(gcf, 'name', 'Environment', 'NumberTitle', 'off')
set(gca, 'YDir', 'reverse')
set(gca, 'ZDir', 'reverse')
set(gcf, 'color', 'w')
title('Environment')
plot3([0, 1], [0, 0], [0, 0], 'r','HandleVisibility','off')
plot3([0, 0], [0, 1], [0, 0], 'b','HandleVisibility','off')
plot3([0, 0], [0, 0], [0, 1], 'g','HandleVisibility','off')
plot3(env(1,:), env(2,:), env(3,:), 'k.', 'MarkerSize', 0.1,'HandleVisibility','off')
plot3(true_state(2,:), true_state(3,:), true_state(4,:), 'b', 'linewidth', 1.5)
if name(1:4) == 'wing'
    est_state = reshape(fread(fopen(strcat(['/tmp/',name,'_ekf_est.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;euler;acc_bias;gyro_bias;wind_inertial;baro_bias]
    plot3(est_state(2,:), est_state(3,:), est_state(4,:), 'r', 'linewidth', 1.5)
elseif name(1:4) == 'quad'
    est_state = reshape(fread(fopen(strcat(['/tmp/',name,'_ekf_est.log']), 'r'), 'double'), 1 + 15, []); % [time;pos;vel;euler;acc_bias;gyro_bias]
    plot3(est_state(2,:), est_state(3,:), est_state(4,:), 'r', 'linewidth', 1.5)
end
plot3([true_state(2,1),cmd_state(2,1)], [true_state(3,1),cmd_state(3,1)], [true_state(4,1),cmd_state(4,1)], 'g--','HandleVisibility','off')
plot3(cmd_state(2,:), cmd_state(3,:), cmd_state(4,:), 'g--')
plot3(bike_state(2,:), bike_state(3,:), bike_state(4,:), 'm', 'linewidth', 1.5)
view(-50, 20)
axis equal
xlabel('North')
ylabel('East')
zlabel('Down')
legend('Air Vehicle Truth','Air Vehicle Est','Air Command','Ground Vehicle')

if params.enable_wind
    vw = reshape(fread(fopen(strcat('/tmp/wind.log'), 'r'), 'double'), 4, []);
    figure()
    set(gcf, 'name', 'Wind', 'NumberTitle', 'off')
    titles = ["North", "East","Down"];
    idx = 1;
    for i=1:3
        subplot(3,1,i), hold on, grid on
        title(titles(i))
        plot(vw(1,:), vw(i + idx, :), 'linewidth', 1.3)
    end
end
