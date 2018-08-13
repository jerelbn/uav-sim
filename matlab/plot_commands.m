%% Plot the throttle commands
figure(f); clf; f=f+1;
set(gcf, 'name', 'Throttle', 'NumberTitle', 'off');
title('Throttle Commands');
plot(command(1,:), command(14, :), 'linewidth', 1.3);

%% Plot the yaw rate command
figure(f); clf; f=f+1;
set(gcf, 'name', 'Yaw Rate Command', 'NumberTitle', 'off');
title('Yaw Rate Commands');
plot(command(1,:), command(13, :), 'linewidth', 1.3);
