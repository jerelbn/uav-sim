% rotation matrix from quaternion
function R = R_from_q(q)
    % unpack elements
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    % pack output
    R = [2*(qw^2+qx^2)-1, 2*(qw*qz+qx*qy), 2*(qx*qz-qw*qy)
         2*(qx*qy-qw*qz), 2*(qw^2+qy^2)-1, 2*(qw*qx+qy*qz)
         2*(qw*qy+qx*qz), 2*(qy*qz-qw*qx), 2*(qw^2+qz^2)-1];
end