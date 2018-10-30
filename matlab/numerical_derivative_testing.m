clear

dt = 0.01;
tf = 7;
t = 0:dt:tf;

fs = sin(t) + 0.001 * randn(size(t));
dfs = cos(t);

dfs_1st = zeros(size(dfs));
for i = 1:length(t)
    if i < length(t)
        dfs_1st(i) = (fs(i+1) - fs(i)) / dt;
    else
        dfs_1st(i) = (fs(i) - fs(i-1)) / dt;
    end
end

dfs_2nd = zeros(size(dfs));
for i = 1:length(t)
    if i == 1
        dfs_2nd(i) = -(3*fs(i) - 4*fs(i+1) + fs(i+2)) / (2*dt);
    elseif i == length(t)
        dfs_2nd(i) = (3*fs(i) - 4*fs(i-1) + fs(i-2)) / (2*dt);
    else
        dfs_2nd(i) = (fs(i+1) - fs(i-1)) / (2*dt);
    end
end

dfs_4th = zeros(size(dfs));
for i = 1:length(t)
    if i < 3
        dfs_4th(i) = -(25*fs(i) - 48*fs(i+1) + 36*fs(i+2) - 16*fs(i+3) + 3*fs(i+4)) / (12*dt);
    elseif i > length(t)-2
        dfs_4th(i) = (25*fs(i) - 48*fs(i-1) + 36*fs(i-2) - 16*fs(i-3) + 3*fs(i-4)) / (12*dt);
    else
        dfs_4th(i) = (8*fs(i+1) - fs(i+2) - 8*fs(i-1) + fs(i-2)) / (12*dt);
    end
end

figure(1); clf; hold on; grid on;
plot(t, dfs, 'b')
plot(t, dfs_1st, 'm-.')
plot(t, dfs_2nd, 'g-.')
plot(t, dfs_4th, 'r--')
legend('truth','1st','2nd','4th')
