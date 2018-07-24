figure(f); clf; hold on; grid on; f=f+1;
set(gcf, 'name', 'Environment', 'NumberTitle', 'off')
set(gca, 'YDir', 'reverse')
set(gca, 'ZDir', 'reverse')
title('Environment')
% Data is in NED, so I'm flipping the Y and Z axes to look right
plot3([0, 1], [0, 0], [0, 0], 'r')
plot3([0, 0], [0, 1], [0, 0], 'b')
plot3([0, 0], [0, 0], [0, 1], 'g')
plot3(env(1,:), env(2,:), env(3,:), 'k.', 'MarkerSize', 0.1)
plot3(true_state(2,:), true_state(3,:), true_state(4,:), 'b', 'linewidth', 1.3)
% plot3(est(2,:), -est(3,:), -est(4,:), '-k')
view(15, 35)
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')