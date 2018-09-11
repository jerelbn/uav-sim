%% Plot the position states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Bicycle Position', 'NumberTitle', 'off');
titles = ["pn", "pe", "pd"];
idx = 1;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(bicycle_state(1,:), bicycle_state(i + idx, :), 'linewidth', 1.3);
    if i < 3
        plot(bicycle_state(1,:), bicycle_command(3+i, :), 'g--')
    end
%     plot(true_state(1,:), command(i + idx, :), 'g--');
%     legend('truth', 'estimate', 'control')
end

%% plot the remaining states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Other Bicycle States', 'NumberTitle', 'off');
titles = ["Heading", "Velocity", "Steering Angle"];
idx = 4;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(bicycle_state(1,:), bicycle_state(i + idx, :), 'linewidth', 1.3);
%     legend('truth', 'estimate')
end
