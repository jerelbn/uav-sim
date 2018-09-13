% pitch from quaternion (angle < 90 deg)
% input must be rotation from fixed-frame to body
function pitch = pitch_from_q(q)
    % unpack elements
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    % pack output
    val = 2*(qw*qy - qx*qz);
    if abs(val) > 1 % hold at 90 deg if invalid
        pitch = sign(val)*pi/2;
    else
        pitch = asin(val);
    end
end

