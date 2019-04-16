% roll from quaternion
% input must be rotation from fixed-frame to body
function roll = roll_from_q(q)
    % unpack elements
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    % pack output
    roll = atan2(2*(qw*qx + qy*qz), 2*(qw^2 + qz^2) - 1);
end

