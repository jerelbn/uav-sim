% Compute true relative states for bearings-only
target_truth_bo = target_est_bo;
for i = 1:size(target_truth_bo,2)
    R_i2b = Rq(aircraft_bo(11:14,i));
    target_truth_bo(2:4,i) = R_i2b*(target(2:4,i) - aircraft_bo(2:4,i));
    if i > 1 && i < size(target_truth_bo,2)
        vt_i = (target(2:4,i+1) - target(2:4,i-1)) / (target(1,i+1) - target(1,i-1));
        target_truth_bo(5:7,i) = R_i2b*vt_i - aircraft_bo(5:7,i);
    end
end
target_truth_bo(5:7,1) = target_truth_bo(5:7,2);
target_truth_bo(5:7,end) = target_truth_bo(5:7,end-1);

% Compute true relative states for range+bearings
target_truth_rb = target_est_bo;
for i = 1:size(target_truth_rb,2)
    R_i2b = Rq(aircraft_rb(11:14,i));
    target_truth_rb(2:4,i) = R_i2b*(target(2:4,i) - aircraft_rb(2:4,i));
    if i > 1 && i < size(target_truth_rb,2)
        vt_i = (target(2:4,i+1) - target(2:4,i-1)) / (target(1,i+1) - target(1,i-1));
        target_truth_rb(5:7,i) = R_i2b*vt_i - aircraft_rb(5:7,i);
    end
end
target_truth_rb(5:7,1) = target_truth_rb(5:7,2);
target_truth_rb(5:7,end) = target_truth_rb(5:7,end-1);

% Compute true and estimated relative radius/altitude for bearings-only
e3 = [0;0;1];
Pr = eye(3) - e3*e3';
Ph = e3*e3';
rh_true_bo = zeros(2,size(target_truth_bo,2));
rh_est_bo = zeros(2,size(target_truth_bo,2));
for i = 1:size(target_truth_bo,2)
    roll = roll_from_q(aircraft_bo(5:8,i));
    pitch = pitch_from_q(aircraft_bo(5:8,i));
    R_l2b = R_v2_to_b(roll)*R_v1_to_v2(pitch);
    rh_true_bo(1,i) = norm(Pr*R_l2b'*target_truth_bo(2:4,i));
    rh_true_bo(2,i) = norm(Ph*R_l2b'*target_truth_bo(2:4,i));
    rh_est_bo(1,i) = norm(Pr*R_l2b'*target_est_bo(2:4,i));
    rh_est_bo(2,i) = norm(Ph*R_l2b'*target_est_bo(2:4,i));
end

% Compute true and estimated relative radius/altitude for range+bearings
rh_true_rb = zeros(2,size(target_truth_rb,2));
rh_est_rb = zeros(2,size(target_truth_rb,2));
for i = 1:size(target_truth_rb,2)
    roll = roll_from_q(aircraft_rb(11:14,i));
    pitch = pitch_from_q(aircraft_rb(11:14,i));
    R_l2b = R_v2_to_b(roll)*R_v1_to_v2(pitch);
    rh_true_rb(1,i) = norm(Pr*R_l2b'*target_truth_rb(2:4,i));
    rh_true_rb(2,i) = norm(Ph*R_l2b'*target_truth_rb(2:4,i));
    rh_est_rb(1,i) = norm(Pr*R_l2b'*target_est_rb(2:4,i));
    rh_est_rb(2,i) = norm(Ph*R_l2b'*target_est_rb(2:4,i));
end

% Rotate true velocity into body-level frame for bearings-only and
% range+bearings
vel_true_bo = zeros(3,size(target_truth_bo,2));
vel_true_rb = zeros(3,size(target_truth_rb,2));
for i = 1:size(target_truth_bo,2)
    roll = roll_from_q(aircraft_bo(11:14,i));
    pitch = pitch_from_q(aircraft_bo(11:14,i));
    R_l2b = R_v2_to_b(roll)*R_v1_to_v2(pitch);
    vel_true_bo(:,i) = R_l2b'*aircraft_bo(9:11,i);
    vel_true_rb(:,i) = R_l2b'*aircraft_rb(9:11,i);
end
