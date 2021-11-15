function [ x_hat, P_hat ] = discrete_kfvn( A, B, C, Q, R, z, u, x0, P0 )
%UNTITLED2 Summary of this function goes here
%   

N = size(z,1);
x_prev = x0;
P_prev = P0;
I = eye(size(P0));

x_hat = [];
P_hat = [];

for i = 1: N
    
    % prediction steps
    x_pred = A * x_prev + B * u;
    P_pred = A * P_prev * A' + Q;
    
    % correction steps
    K = P_pred * C' * inv(C * P_pred * C' + R);
    x_update = x_pred + K * (z(i) - C * x_pred);
    P_update = (I - K * C) * P_pred;
    
    x_hat = [x_hat x_update];
    P_hat = [P_hat P_update];
    x_prev = x_update;
    P_prev = P_update;
    
end

