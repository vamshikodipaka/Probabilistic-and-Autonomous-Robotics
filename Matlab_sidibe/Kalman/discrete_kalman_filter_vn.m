function[x_hat]=discrete_kalman_filter_vn( A, B, C, Q, R, z, u, x0, P0 );
% a discrete kalman filetr which takes input as state space
% equations of systems and measurements, intial state and covariance
N = size(z,1);

x_hat = [];
P = P0;
x = x0;
I = eye(size(P));
for i = 1:N
    
    % prediction steps
    xPred = A * x + B * u;
    PPred = A * P * A' + Q;
    
    % correction steps
    K = PPred * C' * inv(C * PPred * C' + R);
    x = xPred + K*( z(i,:) - C * xPred);
    P = (I - K * C) * PPred;
    
    x_hat = [x_hat x];
    
end
end

