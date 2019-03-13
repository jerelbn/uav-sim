function plot_vehicle(name)

% Load data
true_state = reshape(fread(fopen(strcat(['/tmp/',name,'_true_state.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;accel;att;ang_vel;ang_accel]
true_euler = reshape(fread(fopen(strcat(['/tmp/',name,'_euler_angles.log']), 'r'), 'double'), 1 + 3, []); % [time;roll;pitch;yaw]
commanded_state = reshape(fread(fopen(strcat(['/tmp/',name,'_commanded_state.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;accel;att;ang_vel;ang_accel]
command = reshape(fread(fopen(strcat(['/tmp/',name,'_command.log']), 'r'), 'double'), 1 + 4, []); % [time;aileron;elevator;throttle;rudder]
euler_command = reshape(fread(fopen(strcat(['/tmp/',name,'_euler_command.log']), 'r'), 'double'), 1 + 3, []); % [time;roll;pitch;yaw]


figure()
set(gcf, 'name', 'Position', 'NumberTitle', 'off');
titles = ["North", "East", "Altitude"];
idx = 1;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    if i < 3
        plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 2.0)
        plot(commanded_state(1,:), commanded_state(i + idx, :), 'g--', 'linewidth', 1.5)
    else
        plot(true_state(1,:), -true_state(i + idx, :), 'linewidth', 2.0)
        plot(commanded_state(1,:), -commanded_state(i + idx, :), 'g--', 'linewidth', 1.5)
    end
    if i == 1
        legend('True', 'Command')
    end
end


figure()
set(gcf, 'name', 'Velocity', 'NumberTitle', 'off');
titles = ['u','v','w'];
idx = 4;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 2.0)
    plot(commanded_state(1,:), commanded_state(i + idx, :), 'g--', 'linewidth', 1.5)
    if i == 1
        legend('True', 'Command')
    end
end


figure()
set(gcf, 'name', 'Attitude', 'NumberTitle', 'off');
titles = ["w","x","y","z"];
idx = 10;
for i=1:4
    subplot(4, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 2.0)
    plot(commanded_state(1,:), commanded_state(i + idx, :), 'g--', 'linewidth', 1.5)
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
    plot(true_euler(1,:), true_euler(i + 1, :), 'linewidth', 2.0)
    plot(euler_command(1,:), euler_command(i + 1, :), 'g--', 'linewidth', 1.5)
    if i == 1
        legend('True', 'Command')
    end
end


figure()
set(gcf, 'name', 'Angular Rate', 'NumberTitle', 'off');
titles = ['p','q','r'];
idx = 14;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 2.0)
    plot(commanded_state(1,:), commanded_state(i + idx, :), 'g--', 'linewidth', 1.5)
    if i == 1
        legend('True', 'Command')
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
    plot(command(1,:), command(i + 1, :), 'linewidth', 2.0)
    if i == 3
        ylim([0 1])
    else
        ylim([-1 1])
    end
end
