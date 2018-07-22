% Load truth
file = fopen(strcat(directory,'true_state.bin'), 'r');
true_state = fread(file, 'double');
true_state = reshape(true_state, 17, []);

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
