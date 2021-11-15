%EKF%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% problem1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; close all; clear all;

%parameters
phi1 = 0.5; phi3 = 0.5;
phi2 = 0.2;
omega = 4 * 1e-2;
sig_process = 1e-4; % variance of gaussian process noise
sig_measurement = 1e-1; % variance of gaussian measurement noise
T = 60;  % Number of time steps

x = zeros(T, 1); %The State
y = zeros(T, 1); % The measurements

x(1) = 1;
for t=2:T
    process_noise = sqrt(sig_process) * rand(1,1);
    measurement_noise = sqrt(sig_measurement) * rand(1,1);
    
    x(t) = feval('fstate', x(t-1)) + process_noise;
    y(t) = feval('hmeasurement',x(t-1), t) + measurement_noise;
    
end


% plot the data
figure(1);
plot(1:T, x, 'r' , 1:T, y, 'g');
xlabel('Time');
ylabel('Data');
legend('States(x)', 'Observations(y)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%OPTION 1 Linearised KF
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

A = omega * pi + phi1;
B = 1;
C1 = 2*phi2;
C2 = phi3;
Q = sig_process;
R = sig_measurement;

%use KF
x0 = 1; P0 = 1000; u = 1;
[ x_hat1, P_hat1 ] = discrete_kf( A, B, C1, Q, R, y(1:30), u, x0, P0 );
[ x_hat2, P_hat2 ] = discrete_kf( A, B, C2, Q, R, y(31:end), u, x0, P0 );

x_hat = [x_hat1'; x_hat2']; % estimated state

hold on;
plot(1:T, x_hat, 'b');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OPTION 2 : EKF
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x0 = 1; P0 = 1000; u = 1;

x_hat_ekf = zeros(size(x));
P_hat_ekf = zeros(size(x));

x_hat_ekf(1) = x0;
P_hat_ekf = P0;

for t = 2:T
    
    %prediction
    x_pred = feval('fstate', x_hat_ekf(t-1)); % use state equation with function f
    F = omega * pi * cos(omega*pi*x(t-1)) + phi1;
    P_pred = F * P_hat_ekf(t-1) * F' + Q; % we need the derivative of F
        
    %correction
    y_pred = feval('hmeasurement', x_pred, t);
    if t<= 30
        H = 2 * phi2 * x_pred;
    else
        H = phi3;
    end
    
    K = P_pred  * H' * inv(H * P_pred * H' + R);
    x_hat_ekf(t) = x_pred + K * (y(t) - y_pred);
    P_hat_ekf(t) = P_pred - P_pred * K * H;
    
end  
    
figure(2)
plot(1:T, x, 'r', 1:T, y, 'g', 1:T, x_hat_ekf, 'b');
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Problem2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x = zeros(T, 1); %The State
y = zeros(T, 1); % The measurements

x(1) = 1;

process_noise = gampdf(1:T, 3, 2);

for t=2:T
    
    measurement_noise = sqrt(sig_measurement) * rand(1,1);
    
    x(t) = feval('fstate', x(t-1)) + process_noise(t-1);
    y(t) = feval('hmeasurement',x(t-1), t) + measurement_noise;
    
end

x0 = 1; P0 = 1000; u = 1;

x_hat_ekf = zeros(size(x));
P_hat_ekf = zeros(size(x));

x_hat_ekf(1) = x0;
P_hat_ekf = P0;

for t = 2:T
    
    %prediction
    x_pred = feval('fstate', x_hat_ekf(t-1)); % use state equation with function f
    F = omega * pi * cos(omega*pi*x(t-1)) + phi1;
    P_pred = F * P_hat_ekf(t-1) * F' + Q; % we need the derivative of F
        
    %correction
    y_pred = feval('hmeasurement', x_pred, t);
    if t<= 30
        H = 2 * phi2 * x_pred;
    else
        H = phi3;
    end
    
    K = P_pred  * H' * inv(H * P_pred * H' + R);
    x_hat_ekf(t) = x_pred + K * (y(t) - y_pred);
    P_hat_ekf(t) = P_pred - P_pred * K * H;
    
end  
    
figure(3)
plot(1:T, x, 'r', 1:T, y, 'g', 1:T, x_hat_ekf, 'b');