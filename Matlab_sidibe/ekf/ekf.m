%EKF%
% problem1%

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
    process_noise(t) = sqrt(sig_process * rand(1,1));
    measurement_noise(t) = sqrt(sig_measurement * rand(1,1));
    
    x(t) = 1 + sin(omega*pi*x(t-1)) + phi1*x(t-1) + process_noise(t);
    if t<= 30
        y(t) = phi2*x(t-1)^2 + measurement_noise(t);
    else
        y(t) = phi3*x(t-1) + measurement_noise(t);
    end
end


% plot the data
figure(1);
plot(1:T, x, 'r' , 1:T, y, 'b');
xlabel('Time');
ylabel('Data');
legend('States(x)', 'Observations(y)');

