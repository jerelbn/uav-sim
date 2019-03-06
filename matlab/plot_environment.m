function plot_environment(name)

% Load data
env = reshape(fread(fopen(strcat('/tmp/environment.log'), 'r'), 'double'), 3, []);
vw = reshape(fread(fopen(strcat('/tmp/wind.log'), 'r'), 'double'), 4, []);
air_state = reshape(fread(fopen(strcat(['/tmp/',name,'_true_state.log']), 'r'), 'double'), 1 + 19, []);
air_command = reshape(fread(fopen(strcat(['/tmp/',name,'_commanded_state.log']), 'r'), 'double'), 1 + 19, []);
bike_state = reshape(fread(fopen('/tmp/bike1_true_state.log', 'r'), 'double'), 7, []);


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
plot3(air_state(2,:), air_state(3,:), air_state(4,:), 'b', 'linewidth', 1.5)
plot3([air_state(2,1),air_command(2,1)], [air_state(3,1),air_command(3,1)], [air_state(4,1),air_command(4,1)], 'g--','HandleVisibility','off')
plot3(air_command(2,:), air_command(3,:), air_command(4,:), 'g--')
plot3(bike_state(2,:), bike_state(3,:), bike_state(4,:), 'm', 'linewidth', 1.5)
view(-50, 20)
axis equal
xlabel('North')
ylabel('East')
zlabel('Down')
legend('Air Vehicle','Air Command','Ground Vehicle')


% Plot the wind velocity
figure()
set(gcf, 'name', 'Wind', 'NumberTitle', 'off')
titles = ["North", "East","Down"];
idx = 1;
for i=1:3
    subplot(3,1,i), hold on, grid on
    title(titles(i))
    plot(vw(1,:), vw(i + idx, :), 'linewidth', 1.3)
end
