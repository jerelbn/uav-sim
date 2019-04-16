% rotation from vehicle-1 to vehicle-2 frame
function R = R_v1_to_v2(pitch)
    R = [cos(pitch), 0, -sin(pitch)
                  0, 1, 0
         sin(pitch), 0, cos(pitch)];
end