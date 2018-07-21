%% Truth
% Load truth
file = fopen(strcat(directory,'true_state.bin'), 'r');
true_state = fread(file, 'double');
true_state = reshape(true_state, 17, []);

% Load commands
file = fopen(strcat(directory,'command.bin'), 'r');
command = fread(file, 'double');
command = reshape(command, 14, []);