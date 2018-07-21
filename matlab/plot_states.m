%% Plot the position states
cov_color = 0.5;
sigma = 2;

figure(f); clf; f=f+1;
set(gcf, 'name', 'Position', 'NumberTitle', 'off');
titles = ["pn", "pe", "pd"];
idx = 1;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 1.3);
%     plot(est(1,:), est(i+idx,:));
%     plot(truth(1,:), control(i + idx, :));
    
%     if (plot_covariance)
%         plot(cov(1,:), est(i+idx,:) + sigma * sqrt(cov(i+idx,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
%         plot(cov(1,:), est(i+idx,:) - sigma * sqrt(cov(i+idx,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
%     end
%     legend('truth', 'estimate', 'control')
end

%% plot the attitude states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Attitude', 'NumberTitle', 'off');
titles = ["qw","qx","qy","qz"];
idx = 4;
for i=1:4
    subplot(4, 1, i); hold on;
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 1.3);
%     plot(est(1,:), est(i+idx,:));
%     title(titles(i))
%     if (plot_covariance && i < 4)
%         plot(cov(1,:), est(i+idx,:) + sigma * sqrt(cov(i+idx,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
%         plot(cov(1,:), est(i+idx,:) - sigma * sqrt(cov(i+idx,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
%     end
%     legend('truth', 'estimate')
end

%% Plot the velocity states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Velocity', 'NumberTitle', 'off');
titles = ['u','v','w'];
idx = 8;
for i=1:3
    subplot(3, 1, i); hold on;
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 1.3);
%     plot(est(1,:), est(i+idx,:));
%     title(titles(i))
%     if (plot_covariance)
%         plot(cov(1,:), est(i+idx,:) + sigma * sqrt(cov(i+idx,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
%         plot(cov(1,:), est(i+idx,:) - sigma * sqrt(cov(i+idx,:)), 'color', cov_color*[1, 1, 1], 'linewidth', 0.5)
%     end
%     legend('truth', 'estimate')
end

%% Plot the angular rates
figure(f); clf; f=f+1;
set(gcf, 'name', 'Omega', 'NumberTitle', 'off');
titles = ["wx", "wy","wz"];
idx = 11;
for i=1:3
    subplot(3,1,i); hold on;
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 1.3);
%     plot(u(1,:), u(i+4,:));
%     title(titles(i))
%     legend('truth', 'measured')
end

%% Plot the linear acceleration
figure(f); clf; f=f+1;
set(gcf, 'name', 'Accel', 'NumberTitle', 'off');
titles = ["ax", "ay","az"];
idx = 14;
for i=1:3
    subplot(3,1,i); hold on;
    plot(true_state(1,:), true_state(i + idx, :), 'linewidth', 1.3);
%     plot(u(1,:), u(i+4,:));
%     title(titles(i))
%     legend('truth', 'measured')
end