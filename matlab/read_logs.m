% Load truth
file = fopen(strcat(directory,'true_state.bin'), 'r');
true_state = fread(file, 'double');
true_state = reshape(true_state, 1 + 19, []);

% Load EKF state
file = fopen(strcat(directory,'ekf_state.bin'), 'r');
ekf_state = fread(file, 'double');
ekf_state = reshape(ekf_state, 1 + 19, []);

% Load EKF covariance diagonals
file = fopen(strcat(directory,'ekf_cov.bin'), 'r');
ekf_cov = fread(file, 'double');
ekf_cov = reshape(ekf_cov, 1 + 18, []);

% Load commands
file = fopen(strcat(directory,'command.bin'), 'r');
command = fread(file, 'double');
command = reshape(command, 14, []);

% Load environment
file = fopen(strcat(directory,'environment.bin'), 'r');
env = fread(file, 'double');
env = reshape(env, 3, []);

% Load wind
file = fopen(strcat(directory,'wind.bin'), 'r');
vw = fread(file, 'double');
vw = reshape(vw, 4, []);

% Load accelerometer measurements
file = fopen(strcat(directory,'accel.bin'), 'r');
accel = fread(file, 'double');
accel = reshape(accel, 10, []);
accel_bias = accel(5:7,:);
accel_noise = accel(8:10,:);

% Load rate gyro measurements
file = fopen(strcat(directory,'gyro.bin'), 'r');
gyro = fread(file, 'double');
gyro = reshape(gyro, 10, []);
gyro_bias = gyro(5:7,:);
gyro_noise = gyro(8:10,:);

% Load pixel measurements
file = fopen(strcat(directory,'camera.bin'), 'r');
pix = fread(file, 'double');
pix = reshape(pix, 15001, []);

% Compute heading truth and estimates
true_heading = zeros(1,size(true_state,2));
ekf_heading = zeros(1,size(ekf_state,2));
for i = 1:size(true_heading,2)
    true_heading(i) = yaw_from_q(true_state(5:8,i));
    ekf_heading(i) = yaw_from_q(ekf_state(5:8,i));
end
