clear, close all

rho = 1.2682;
S_prop = 0.0314;
C_prop = 1.0;
k_motor = 40;
C_F_t = 19.2089;

Va = linspace(0,30,100);
delta_t = linspace(0,1,100);

[Va_,delta_t_] = meshgrid(Va,delta_t);
% thrust = 0.5 * rho * S_prop * C_prop * ((k_motor * delta_t_).^2 - Va_.^2);
thrust = rho * S_prop * C_prop * (Va_ + delta_t_ .* (k_motor - Va_)) .* delta_t_ .* (k_motor - Va_);
thrust_linear = C_F_t * delta_t_;

figure(1), clf, grid on
surf(Va, delta_t, thrust, 'facecolor', 'b', 'facealpha', 0.5), hold on
surf(Va, delta_t, thrust_linear, 'facecolor', 'r', 'facealpha', 0.5)
xlabel('V_a')
ylabel('\delta_t')
zlabel('Thrust')

% figure(2), clf, grid on
% plot(delta_t_(:,1), thrust(:,50))
% xlabel('\delta_t')
% ylabel('Thrust')
