% Analytical sampson's error in terms of R and Rt
clear
addpath '../../code/matlab/common/'

syms R11 R12 R13 R21 R22 R23 R31 R32 R33
syms Rt11 Rt12 Rt13 Rt21 Rt22 Rt23 Rt31 Rt32 Rt33
syms ek1 ek2 ek3
syms ec1 ec2 ec3

R = [R11 R12 R13
     R21 R22 R23
     R31 R32 R33];
Rt = [Rt11 Rt12 Rt13
      Rt21 Rt22 Rt23
      Rt31 Rt32 Rt33];
ek = [ek1;ek2;ek3];
ec = [ec1;ec2;ec3];
e1 = [1;0;0];
e2 = [0;1;0];
e3 = [0;0;1];

E = R * skew(Rt * e3);
lkTE = transpose(ek) * E;
Elc = E * ec;
S = transpose(ek) * E * ec / sqrt((lkTE * e1)^2 + (lkTE * e2)^2 + (transpose(e1) * Elc)^2 + (transpose(e2) * Elc)^2);
disp(simplify(S))