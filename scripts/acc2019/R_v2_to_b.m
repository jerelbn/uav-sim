% rotation from vehicle-2 to body frame
function R = R_v2_to_b(roll)
    R = [1,          0, 0
         0,  cos(roll), sin(roll)
         0, -sin(roll), cos(roll)];
end