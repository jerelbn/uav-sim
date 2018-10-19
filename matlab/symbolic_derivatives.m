% Symbolic stuff
clear
addpath ../../code/matlab/common/

syms w x y z d1 d2

e1 = [1;0;0];
e2 = [0;1;0];

delta = [d1;d2];
q = [w;x;y;z];
R = [2*(w^2+x^2)-1, 2*(w*z+x*y), 2*(x*z-w*y)
     2*(x*y-w*z), 2*(w^2+y^2)-1, 2*(w*x+y*z)
     2*(w*y+x*z), 2*(y*z-w*x), 2*(w^2+z^2)-1];
T = transpose(R) * [e1 e2];

dq = 1/2 * [2;T*delta];
dq_x_q = simplify(qmul(dq,q))


% testing
q = randn(4,1);
q = q / norm(q);
qw = q(1);
qx = q(2);
qy = q(3);
qz = q(4);
dqpd_dd = 0.5 * [-qx -qy -qz
                  qw -qz  qy
                  qz  qw -qx
                 -qy  qx  qw];
dqpd_ddi = pinv(dqpd_dd);
ddq_ddq = dqpd_ddi * dqpd_dd