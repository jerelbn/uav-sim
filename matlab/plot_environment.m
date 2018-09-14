figure(f); clf; hold on; grid on; f=f+1;
set(gcf, 'name', 'Environment', 'NumberTitle', 'off')
set(gca, 'YDir', 'reverse')
set(gca, 'ZDir', 'reverse')
set(gcf, 'color', 'w')
title('Environment')
plot3([0, 1], [0, 0], [0, 0], 'r')
plot3([0, 0], [0, 1], [0, 0], 'b')
plot3([0, 0], [0, 0], [0, 1], 'g')
% plot3(env(1,:), env(2,:), env(3,:), 'k.', 'MarkerSize', 0.1)
plot3(true_state(2,:), true_state(3,:), true_state(4,:), 'b', 'linewidth', 1.3)
% plot3(ekf_state(2,:), ekf_state(3,:), ekf_state(4,:), 'r')
plot3(command(2,:), command(3,:), command(4,:), 'g--')
plot3(bicycle_state(2,:), bicycle_state(3,:), bicycle_state(4,:), 'm', 'linewidth', 1.3)
view(-50, 20)
axis equal
xlabel('North')
ylabel('East')
zlabel('Down')
legend('Multi-rotor','Ground Vehicle')