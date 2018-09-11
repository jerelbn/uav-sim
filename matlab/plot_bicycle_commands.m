%% Plot the force commands
figure(f); clf; f=f+1;
set(gcf, 'name', 'Force', 'NumberTitle', 'off');
title('Throttle Commands');
plot(bicycle_command(1,:), bicycle_command(2, :), 'linewidth', 1.3);

%% Plot the yaw rate command
figure(f); clf; f=f+1;
set(gcf, 'name', 'Torque', 'NumberTitle', 'off');
title('Yaw Rate Commands');
plot(bicycle_command(1,:), bicycle_command(3, :), 'linewidth', 1.3);
