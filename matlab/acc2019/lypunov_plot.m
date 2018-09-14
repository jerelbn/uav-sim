% Plot Lypunov Function
clear
close all

r_tilde = -2:0.1:2;
h_tilde = -2:0.1:2;
kr = 1;
kh = 1;
vt = randn(3,1);
vt = 2.0 * vt / norm(vt);
er = [randn(2,1);0];
er = er / norm(er);
e3 = [0;0;1];
V = zeros(length(r_tilde),length(h_tilde));
for i = 1:length(r_tilde)
    for j = 1:length(h_tilde)
        V(i,j) = -kr*r_tilde(i)^2 - kh*h_tilde(j)^2 + (r_tilde(i)*er' + h_tilde(j)*e3')*vt;
    end
end
[X,Y] = meshgrid(r_tilde,h_tilde);
figure(1), clf
set(gcf,'color','w')
surf(X,Y,V)
% shading interp
shading flat
colorbar
xlabel('$\tilde r$','Interpreter','latex','fontsize',20)
ylabel('$\tilde h$','Interpreter','latex','fontsize',20)
zlabel('$\dot V$','Interpreter','latex','fontsize',20)
title('Surface of the Lypunov Function Derivative')
