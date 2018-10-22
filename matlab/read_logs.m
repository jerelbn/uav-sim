% Load truth
file = fopen(strcat(directory,'true_state.bin'), 'r');
true_state = fread(file, 'double');
true_state = reshape(true_state, 1 + 19, []);

% Load true global Euler angles
file = fopen(strcat(directory,'true_global_euler.bin'), 'r');
true_global_euler = fread(file, 'double');
true_global_euler = reshape(true_global_euler, 1 + 3, []);

% Load body to IMU pose
file = fopen(strcat(directory,'true_pose_b2u.bin'), 'r');
pose_b2u = fread(file, 'double');

% Load EKF state
file = fopen(strcat(directory,'ekf_state.bin'), 'r');
ekf_state = fread(file, 'double');
ekf_state = reshape(ekf_state, 1 + 16 + 7, []);

% Load EKF covariance diagonals
file = fopen(strcat(directory,'ekf_cov.bin'), 'r');
ekf_cov = fread(file, 'double');
ekf_cov = reshape(ekf_cov, 1 + 15 + 6, []);

% Load EKF global position and Euler angles
file = fopen(strcat(directory,'ekf_global_pos_euler.bin'), 'r');
ekf_global_pos_euler = fread(file, 'double');
ekf_global_pos_euler = reshape(ekf_global_pos_euler, 1 + 3 + 3, []);

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

% Load motion capture measurements
file = fopen(strcat(directory,'mocap.bin'), 'r');
mocap = fread(file, 'double');
mocap = reshape(mocap, 21, []);

% Load bicycle truth
file = fopen(strcat(directory,'bicycle_true_state.bin'), 'r');
bicycle_state = fread(file, 'double');
bicycle_state = reshape(bicycle_state, 7, []);

% Load bicycle commands
file = fopen(strcat(directory,'bicycle_command.bin'), 'r');
bicycle_command = fread(file, 'double');
bicycle_command = reshape(bicycle_command, 5, []);

% Load target estimates
file = fopen(strcat(directory,'target.bin'), 'r');
target_est = fread(file, 'double');
target_est = reshape(target_est, 7, []);
