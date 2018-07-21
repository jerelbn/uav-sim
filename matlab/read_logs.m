%% Truth
% Load truth
file = fopen(strcat(directory,'true_state.bin'), 'r');
true_state = fread(file, 'double');
true_state = reshape(true_state, 17, []);
