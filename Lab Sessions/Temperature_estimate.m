% Single unknown estimate with linear KF 
%
% Estimate the temperature of a plant from the noisy measurements of a 
% thermometer using a linear Kalman Filter
%
% Programmed by Joaquim Salvi

clear all;
NumberTimeStamps = 500;

%%% FIXED TEMPERATURE
% Temperature of the plant to be estimated
% Real_temperature = 30;
% Definition of the real state
% x(1:NumberTimeStamps)= Real_temperature; 

t = linspace(0,2*pi,NumberTimeStamps);
Real_temperature = 50 + 10*sin(2*pi*2*t);
x = Real_temperature;

% Definition of process and measurement noises
r = 10;  % measurement noise variance, change to see effect
pn = 1.5;  % process noise variance, change to see effect

% Definition Measurement matrix
% The Measurement matrix is a diagonal of ones, which means that the 
% measurement is a direct reading of the state, only influenced by noise.
H = diag(ones(1,1)); 

% Definition of the State matrix
% The State matrix is a diagonal of ones, which means that the prediction
% of next state is equivalent to current state.
Fk = diag(ones(1,1));   

% Definition of the Control vector
% The control vector is null, which means that the are no external inputs
% that change the state.
Uk = zeros(1,1);   

% Definition of the Covariance matrix
% The initial covariance matrix is defined by a diagonal of ones, which 
% means that the state is very related to the state itself.
Pk(1) = diag(ones(1,1)); 

%Definition of the Process Noise Matrix
Q = pn * diag(ones(1,1)); 
%Definition of the Measurement Noise Matrix
R = r * diag(ones(1,1));

% Initial prediction of the vector state
N = normrnd(0,sqrt(r));
xhat(1)= H*x(1) + N; % Initial state is unknown, set to the first measurement

for k = 1:NumberTimeStamps-1
    
    % Prediction
    xhat(k+1) = Fk*xhat(k)+Uk;  % Prediction of next state
    zhat(k+1) = H*xhat(k+1);            % Measure at the predicted state
    Pk(k+1) = Fk * Pk(k) *Fk' + Q;
    
    % Observation
    N = normrnd(0,sqrt(r));     % Normal distribution with 0 mean and r standard deviation
    z(k+1) = H * x(k) + N;              % Measure at the correct state
    vv(k+1) = z(k+1) - zhat(k+1);            % Innovation vector, i.e. discrepancy between measures
                                    
    S = H * Pk(k+1) * H' + R;
    
    % update
    W = Pk(k+1) * H' * inv(S);
    xhat(k+1) = xhat(k+1) + W*vv(k+1);
    Pk(k+1) = Pk(k+1) - W * H * Pk(k+1);
end;

% begin plotting
figure; title('Temperature Estimate with KF');
ax=[1 NumberTimeStamps 0 100];
axis(ax); hold on;
plot(x,'g');
plot(xhat,'r');
plot(z,'m.');
for i = 1:NumberTimeStamps
    Uncertainity1(i) = xhat(i)+2*sqrt(Pk(i));
    Uncertainity2(i) = xhat(i)-2*sqrt(Pk(i));
end
plot(Uncertainity1,'y');
plot(Uncertainity2,'y');
% end plotting