% Symbolic stuff
clear
clc
addpath ../../code/matlab/common/

syms w x y z d1 d2
syms w1 x1 y1 z1 w2 x2 y2 z2
syms tx1 ty1 tz1 tx2 ty2 tz2

e1 = [1;0;0];
e2 = [0;1;0];
e3 = [0;0;1];

delta = [d1;d2];
q = [w;x;y;z];

q1 = [w1;x1;y1;z1];
q2 = [w2;x2;y2;z2];

R = [2*(w^2+x^2)-1, 2*(w*z+x*y), 2*(x*z-w*y)
     2*(x*y-w*z), 2*(w^2+y^2)-1, 2*(w*x+y*z)
     2*(w*y+x*z), 2*(y*z-w*x), 2*(w^2+z^2)-1];
R1 = [2*(w1^2+x1^2)-1, 2*(w1*z1+x1*y1), 2*(x1*z1-w1*y1)
      2*(x1*y1-w1*z1), 2*(w1^2+y1^2)-1, 2*(w1*x1+y1*z1)
      2*(w1*y1+x1*z1), 2*(y1*z1-w1*x1), 2*(w1^2+z1^2)-1];
R2 = [2*(w2^2+x2^2)-1, 2*(w2*z2+x2*y2), 2*(x2*z2-w2*y2)
      2*(x2*y2-w2*z2), 2*(w2^2+y2^2)-1, 2*(w2*x2+y2*z2)
      2*(w2*y2+x2*z2), 2*(y2*z2-w2*x2), 2*(w2^2+z2^2)-1];
  
t1 = [tx1;ty1;tz1];
t2 = [tx2;ty2;tz2];



% derivative of q plus delta w.r.t. delta
T = transpose(R) * [e1 e2];
dq = 1/2 * [2;T*delta];
dq_x_q = simplify(qmul(dq,q));


% derivative of q1 minus q2 w.r.t. q1
q3 = qmul(qinv(q2),q1);
q3bar = q3(2:4);

% derivative of T1 minus T2 w.r.t. T1
t3 = simplify(-R2 * (t2 + t1))
q3 = qmul(qinv(q2),q1);
q3bar = simplify(q3(2:4))

% derivative of qt1 minus qt2 w.r.t.qt1
T1 = R1 * [e1 e2];
z1 = transpose(R1) * e3;
z2 = transpose(R2) * e3;
theta = acos(transpose(z2)*z1);
phi = 1/norm(cross(z2,z1));
s = cross(z2,z1);

T1Ts = simplify(transpose(T1)*s);