% Load target truth
file = fopen(strcat(directory,'target.bin'), 'r');
target = fread(file, 'double');
target = reshape(target, 7, []);

% Load bearings-only aircraft truth
file = fopen(strcat(directory,'aircraft_bo.bin'), 'r');
aircraft_bo = fread(file, 'double');
aircraft_bo = reshape(aircraft_bo, 1 + 19, []);

% Load range+bearings aircraft truth
file = fopen(strcat(directory,'aircraft_rb.bin'), 'r');
aircraft_rb = fread(file, 'double');
aircraft_rb = reshape(aircraft_rb, 1 + 19, []);

% Load bearings-only target estimates
file = fopen(strcat(directory,'relative_estimates_bo.bin'), 'r');
target_est_bo = fread(file, 'double');
target_est_bo = reshape(target_est_bo, 7, []);

% Load range+bearings target estimates
file = fopen(strcat(directory,'relative_estimates_rb.bin'), 'r');
target_est_rb = fread(file, 'double');
target_est_rb = reshape(target_est_rb, 7, []);

% Load bearings-only aircraft velocity commands
file = fopen(strcat(directory,'command_bo.bin'), 'r');
command_bo = fread(file, 'double');
command_bo = reshape(command_bo, 14, []);

% Load range+bearings aircraft velocity commands
file = fopen(strcat(directory,'command_rb.bin'), 'r');
command_rb = fread(file, 'double');
command_rb = reshape(command_rb, 14, []);
