% Prepare Variables for Plotting
clear, close all
format compact
set(0,'DefaultFigureWindowStyle','normal')
addpath('../lib/yamlmatlab')
params = ReadYaml('../params/fixed_wing1.yaml');
skip = 300;
fig_width = 600;
fig_x_scale = 1;
fig_y_scale = 1;
num_runs = 100;

% Get waypoints into array for plotting line paths
command = zeros(1,length(params.waypoints));
for i = 1:length(params.waypoints)
    command(i) = params.waypoints{i};
end
command = reshape(command,3,[]);
command = [command, command(:,1)];


% Plot 3D path
fig = figure('Renderer', 'painters', 'Position', [10 10 fig_width fig_width*2/3]);
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_x_scale*fig_pos(3) fig_y_scale*fig_pos(4)];
hold on, grid on
plot3(command(1,:), command(2,:), -command(3,:), 'r-.', 'linewidth', 2.0)
for j = 1:num_runs
    truth = reshape(fread(fopen(strcat(['/tmp/fw_monte_carlo_truth_',int2str(j),'.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;accel;att;ang_vel;ang_accel]
    plot3(truth(2,1:skip:end), truth(3,1:skip:end), -truth(4,1:skip:end), 'b', 'linewidth', 1.0)
end
plot3(command(1,:), command(2,:), -command(3,:), 'r-.', 'linewidth', 2.0)
set(gcf, 'name', '3D Path', 'NumberTitle', 'off')
set(gca, 'YDir', 'reverse')
set(gcf, 'color', 'w')
% title('3D Path')
view(-10, 3)
axis equal
xlabel('North (m)')
ylabel('East (m)')
zlabel('Altitude (m)')
% legend('Commanded Path','Actual Paths')
print(gcf, '~/Dropbox/dev/fw_lqr_paper/figures/3d_path','-dpdf')


% Plot top-down view
fig = figure('Renderer', 'painters', 'Position', [10 10 fig_width fig_width*2/3]);
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_x_scale*fig_pos(3) fig_y_scale*fig_pos(4)];
hold on, grid on
plot(command(2,:), command(1,:), 'r-.', 'linewidth', 2.0)
for j = 1:num_runs
    truth = reshape(fread(fopen(strcat(['/tmp/fw_monte_carlo_truth_',int2str(j),'.log']), 'r'), 'double'), 1 + 19, []); % [time;pos;vel;accel;att;ang_vel;ang_accel]
    plot(truth(3,1:skip:end), truth(2,1:skip:end), 'b', 'linewidth', 1.0)
end
plot(command(2,:), command(1,:), 'r-.', 'linewidth', 2.0)
set(gcf, 'name', 'Top-Down View', 'NumberTitle', 'off')
set(gcf, 'color', 'w')
% title('Top-Down View')
axis equal
xlabel('East (m)')
ylabel('North (m)')
% legend('Commanded Path','Actual Paths','location','west')
print(gcf, '~/Dropbox/dev/fw_lqr_paper/figures/top_down','-dpdf')