function plot_bicycle(name)

state = reshape(fread(fopen(strcat(['/tmp/',name,'_true_state.log']), 'r'), 'double'), 7, []);
command = reshape(fread(fopen(strcat(['/tmp/',name,'_command.log']), 'r'), 'double'), 5, []);

% Plot the position states
figure()
set(gcf, 'name', 'Bicycle Position', 'NumberTitle', 'off');
titles = ["pn", "pe", "pd"];
idx = 1;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(state(1,:), state(i + idx, :), 'linewidth', 1.3)
    if i < 3
        plot(state(1,:), command(3+i, :), 'g--')
    end
end

% plot the remaining states
figure()
set(gcf, 'name', 'Other Bicycle States', 'NumberTitle', 'off');
titles = ["Velocity", "Heading", "Steering Angle"];
idx = 4;
for i=1:3
    subplot(3, 1, i), hold on, grid on
    title(titles(i))
    plot(state(1,:), state(i + idx, :), 'linewidth', 1.3)
end

% Plot the force and torque commands
figure()
set(gcf, 'name', 'Force and Torque', 'NumberTitle', 'off');
titles = ["Force Command", "Torque Command"];
for i=1:2
    subplot(2, 1, i), hold on, grid on
    title(titles(i))
    plot(command(1,:), command(i + 1, :), 'linewidth', 1.3)
end