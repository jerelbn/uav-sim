% yaw from quaternion
% input must be rotation from fixed-frame to body
function yaw = yaw_from_q(q)
    % unpack elements
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    % pack output
    yaw = atan2(2*(qw*qz + qx*qy), 2*(qw^2 + qx^2) - 1);
end

