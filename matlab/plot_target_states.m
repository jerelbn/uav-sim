%% Compute true relative states
target_truth = target_est;
for i = 1:size(target_truth,2)
    target_truth(2:4,i) = Rq(true_state(5:8,i))*(bicycle_state(2:4,i) - true_state(2:4,i));
    if i > 1 && i < size(target_truth,2)
        vt_i = (bicycle_state(2:4,i+1) - bicycle_state(2:4,i-1)) / (bicycle_state(1,i+1) - bicycle_state(1,i-1));
        target_truth(5:7,i) = Rq(true_state(5:8,i))*vt_i - true_state(9:11,i);
%         target_truth(5:7,i) = vt_i;
    else
        target_truth(5:7,i) = zeros(3,1);
    end
%     target_truth(2:4,i) = bicycle_state(2:4,i);
%     target_est(2:4,i) = Rq(true_state(5:8,i))'*target_est(2:4,i) + true_state(2:4,i);
%     target_est(5:7,i) = Rq(true_state(5:8,i))'*(target_est(5:7,i) + true_state(9:11,i));
end

%% Plot relative target  position states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Relative Target Position', 'NumberTitle', 'off');
titles = ["p_x", "p_y", "p_z","heading"];
idx = 1;
for i=1:3
    subplot(4, 1, i); hold on;
    title(titles(i));
    plot(target_truth(1,:), target_truth(i + idx, :), 'linewidth', 1.3);
    plot(target_est(1,:), target_est(i+idx,:), 'r');
    legend('truth', 'estimate')
end

%% Plot the relative target velocity states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Relative Target Velocity', 'NumberTitle', 'off');
titles = ["v_x","v_y","v_z"];
idx = 4;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(target_truth(1,:), target_truth(i + idx, :), 'linewidth', 1.3);
    plot(target_est(1,:), target_est(i+idx,:), 'r');
    legend('truth', 'estimate')
end
