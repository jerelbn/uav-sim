%% Compute true relative states
target_truth = zeros(size(true_state,2));
for i = 1:size(target_truth,2)
    R_i2b = Rq(true_state(11:14,i));
    target_truth(2:4,i) = R_i2b*(bicycle_state(2:4,i) - true_state(2:4,i));
    if i > 1 && i < size(target_truth,2)
        vt_i = (bicycle_state(2:4,i+1) - bicycle_state(2:4,i-1)) / (bicycle_state(1,i+1) - bicycle_state(1,i-1));
        target_truth(5:7,i) = R_i2b*vt_i - true_state(5:7,i);
    end
end
target_truth(5:7,1) = target_truth(5:7,2);
target_truth(5:7,end) = target_truth(5:7,end-1);

%% Plot relative target  position states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Relative Target Position', 'NumberTitle', 'off');
titles = ["p_x", "p_y", "p_z","heading"];
idx = 1;
for i=1:3
    subplot(4, 1, i); hold on;
    title(titles(i));
    plot(target_truth(1,:), target_truth(i + idx, :), 'linewidth', 1.3);
    legend('truth')
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
    legend('truth')
end
