function [ x_hat, phat ] = discrete_kalman_filter( A, B, C, Q, R, z, u, x0, p0 )
% a discrete kalman filetr which takes input as state space 
% equations of systems and measurements, intial state and covariance

x_hat(1) = x0;
P(1) = p0;

for k= 1: length(z)-1
    
    % prediction steps
    x_hat_pred(k+1) = A * x_hat(k) + B * u(k);
    P_pred(k+1) = A * P(k) * A' + Q;
    
    % correction steps
    K(k+1) = P_pred(k+1) * C' * inv(C * P_pred(k+1) * C' + R);
    x_hat(k+1) = x_hat_pred(k+1) + K(k+1) * (z(k+1) - C * x_hat_pred(k+1));
    P(k+1) = (1 - K(k+1) * C) * P_pred(k+1);

end

