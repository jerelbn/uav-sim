%% Plotting for motion capture optimization simulation
clear
format compact
set(0,'DefaultFigureWindowStyle','docked')
f = 1; % Starts figure numbering
state_size = 1 + 16 + 1 + 7 + 4 + 1; % [t;p;v;q;ba;bg;Tm;q_bu;tm]

%% Run Simulation
!cd ../build && ./uav_sim
!cd ../build && ./mocap_opt

%% Read logs
directory = '../logs/';

% Load truth
file = fopen(strcat(directory,'mocap_opt_truth.bin'), 'r');
truth = fread(file, 'double');
truth = reshape(truth, state_size, []);

% Load initial estimate
file = fopen(strcat(directory,'mocap_opt_initial.bin'), 'r');
initial = fread(file, 'double');
initial = reshape(initial, state_size, []);

% Load final estimate
file = fopen(strcat(directory,'mocap_opt_final.bin'), 'r');
final = fread(file, 'double');
final = reshape(final, state_size, []);

%% Plot position states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Position', 'NumberTitle', 'off');
titles = ["pn", "pe", "pd"];
idx = 1;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(truth(1,:), truth(i + idx, :), 'linewidth', 1.3);
    plot(initial(1,:), initial(i+idx,:), 'r-.');
    plot(final(1,:), final(i + idx, :), 'g--');
    if i == 1
        legend('truth', 'initial', 'final')
    end
end

%% Plot velocity states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Velocity', 'NumberTitle', 'off');
titles = ["vx", "vy", "vz"];
idx = 4;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(truth(1,:), truth(i + idx, :), 'linewidth', 1.3);
    plot(initial(1,:), initial(i+idx,:), 'r-.');
    plot(final(1,:), final(i + idx, :), 'g--');
    if i == 1
        legend('truth', 'initial', 'final')
    end
end

%% Plot attitude states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Attitude', 'NumberTitle', 'off');
titles = ["qw", "qx", "qy", "qz"];
idx = 7;
for i=1:4
    subplot(4, 1, i); hold on;
    title(titles(i));
    plot(truth(1,:), truth(i + idx, :), 'linewidth', 1.3);
    plot(initial(1,:), initial(i+idx,:), 'r-.');
    plot(final(1,:), final(i + idx, :), 'g--');
    if i == 1
        legend('truth', 'initial', 'final')
    end
end

%% Plot accel bias states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Accel Bias', 'NumberTitle', 'off');
titles = ["ax", "ay", "az"];
idx = 11;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(truth(1,:), truth(i + idx, :), 'linewidth', 1.3);
    plot(initial(1,:), initial(i+idx,:), 'r-.');
    plot(final(1,:), final(i + idx, :), 'g--');
    if i == 1
        legend('truth', 'initial', 'final')
    end
end

%% Plot gyro bias states
figure(f); clf; f=f+1;
set(gcf, 'name', 'Gyro Bias', 'NumberTitle', 'off');
titles = ["gx", "gy", "gz"];
idx = 14;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(truth(1,:), truth(i + idx, :), 'linewidth', 1.3);
    plot(initial(1,:), initial(i+idx,:), 'r-.');
    plot(final(1,:), final(i + idx, :), 'g--');
    if i == 1
        legend('truth', 'initial', 'final')
    end
end

%% Plot drag coefficient state
figure(f); clf; f=f+1;
set(gcf, 'name', 'Drag', 'NumberTitle', 'off');
titles = ["drag coeff"];
idx = 17;
for i=1:1
    subplot(1, 1, i); hold on;
    title(titles(i));
    plot(truth(1,:), truth(i + idx, :), 'linewidth', 1.3);
    plot(initial(1,:), initial(i+idx,:), 'r-.');
    plot(final(1,:), final(i + idx, :), 'g--');
    if i == 1
        legend('truth', 'initial', 'final')
    end
end

%% Plot body to mocap pose
figure(f); clf; f=f+1;
set(gcf, 'name', 'B2M Position', 'NumberTitle', 'off');
titles = ["px", "py", "pz"];
idx = 18;
for i=1:3
    subplot(3, 1, i); hold on;
    title(titles(i));
    plot(truth(1,:), truth(i + idx, :), 'linewidth', 1.3);
    plot(initial(1,:), initial(i+idx,:), 'r-.');
    plot(final(1,:), final(i + idx, :), 'g--');
    if i == 1
        legend('truth', 'initial', 'final')
    end
end

figure(f); clf; f=f+1;
set(gcf, 'name', 'B2M Rotation', 'NumberTitle', 'off');
titles = ["qw", "qx", "qy", "qz"];
idx = 21;
for i=1:4
    subplot(4, 1, i); hold on;
    title(titles(i));
    plot(truth(1,:), truth(i + idx, :), 'linewidth', 1.3);
    plot(initial(1,:), initial(i+idx,:), 'r-.');
    plot(final(1,:), final(i + idx, :), 'g--');
    if i == 1
        legend('truth', 'initial', 'final')
    end
end

%% Plot body to IMU rotation
figure(f); clf; f=f+1;
set(gcf, 'name', 'B2IMU Rotation', 'NumberTitle', 'off');
titles = ["qw", "qx", "qy", "qz"];
idx = 25;
for i=1:4
    subplot(4, 1, i); hold on;
    title(titles(i));
    plot(truth(1,:), truth(i + idx, :), 'linewidth', 1.3);
    plot(initial(1,:), initial(i+idx,:), 'r-.');
    plot(final(1,:), final(i + idx, :), 'g--');
    if i == 1
        legend('truth', 'initial', 'final')
    end
end

%% Plot motion capture time offset state
figure(f); clf; f=f+1;
set(gcf, 'name', 'Mocap Time Offset', 'NumberTitle', 'off');
titles = ["Mocap Time Offset"];
idx = 29;
for i=1:1
    subplot(1, 1, i); hold on;
    title(titles(i));
    plot(truth(1,:), truth(i + idx, :), 'linewidth', 1.3);
    plot(initial(1,:), initial(i+idx,:), 'r-.');
    plot(final(1,:), final(i + idx, :), 'g--');
    if i == 1
        legend('truth', 'initial', 'final')
    end
end
