% Symbolic stuff
clear
clc
addpath ../../code/matlab/common/

% quaternion representing rotation
q = Quaternion("random");
dqpd_dd = 0.5*[-q.x, -q.y, -q.z
                q.w, -q.z,  q.y
                q.z,  q.w, -q.x
               -q.y,  q.x,  q.w];
ddq_dq = 2 * [-q.x,  q.w,  q.z, -q.y
              -q.y, -q.z,  q.w,  q.x
              -q.z,  q.y, -q.x,  q.w]
ddq_dq1 = 4 * dqpd_dd'

% quaternion representing a unit vector
tvec = randn(3,1);
tvec = tvec / norm(tvec);
q = Quaternion(tvec);
aa = 0.5 - q.w^2 - q.x^2 - q.y^2 - q.z^2;
dqtpd_dd = [  q.x*aa,   q.y*aa
             -q.w*aa, -0.5*q.z
             0.5*q.z,  -q.w*aa
            -0.5*q.y,  0.5*q.x]
ddqt_dqt = 4 * dqtpd_dd'