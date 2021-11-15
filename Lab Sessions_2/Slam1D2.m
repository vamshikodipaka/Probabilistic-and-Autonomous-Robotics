% 1D SLAM with linear KF - Moving vehicle - Absolute measurement
%
% A 1 DOF mobile robot is moving along a straigth lane detecting some
% motionless landmarks; the position/velocity of the robot and position
% of landmarks are computed by means of Simultaneos Localization and 
% Mapping using a linear Kalman Filter.  
% All the measures are absolute.
%
% Programmed by Joaquim Salvi

clear all;
NumberTimeStamps = 1000;
MapDimension = [1,100];
NumberLandmarks = 10;

% landmark positions
p = [10;12;19;31;42;50;60;73;80;91];

% Definition of the real trajectory; vehicle with constant velocity
vel = (MapDimension(2)/NumberTimeStamps);
vel= 0.01;
pos = 25;
x(1)=pos; v(1) = vel;
for k = 2:NumberTimeStamps
    x(k)= x(k-1) + vel;
    v(k)= vel;
end

% Definition of process and measurement noises
r = (15*0.5)^2;  % landmark measurement noise, change to see effect
rx = (8*0.5)^2; % position measurement noise, change to see effect
rv = (1*0.5)^2; % velocity measurement noise, change to see effect
pn = (0.0001)^2; % process noise

% Definition of landmark Measurement matrix
% Landmark distance is absolute
H = [0*ones(10,1) zeros(10,1) 1*diag(ones(1,10))];
H = [1 zeros(1,11);0 1 zeros(1,10); H];

% Definition of the State matrix
% The State matrix is a diagonal of ones, which means that the prediction
% of next state is equivalent to current state, except the position which
% is updated with the current velocity at every time stamp.
Fk = diag(ones(1,12));    Fk(1,2)=1;    

% Definition of the Control vector
% The control vector is null, which means that the are no external inputs
% that change the robot state (velocity is constant, position is constant)
Uk = zeros(12,1);  

% Definition of the Covariance matrix
% The initial covariance matrix is defined with a large diagonal uncertainity
% in both state of the robot and position of the landmarks and with an
% equivalent uncertainity which means that none prevail over the other.
Pk = r*diag(ones(1,12));
Pk(1,1) = rx+rv; Pk(2,2) = rv; Pk(1,2) = rv; Pk(2,1) = rv;

%Definition of the Process/Measure variance
Q = diag([pn*ones(1,2) 0*pn*ones(1,10)]); % landmarks are montionless
%Definition of the Measurement Noise Matrix
R = diag([rx*ones(1,1) rv*ones(1,1) r*ones(1,10)]); 

% Initial prediction of the vector state
xhat(:,1)= [pos vel p']';
N = sqrt(r)*randn(2,1); xhat(1:2,1) = xhat(1:2,1) + N; % adding process noise

for k = 1:NumberTimeStamps-1
    
    % Prediction
    xhat(:,k+1)=Fk*xhat(:,k)+Uk;    % Prediction of next state
    zhat(:,k+1)= H*[xhat(1:2,k+1); xhat(3:12,k+1)]; % Measure at the predicted position
    Pk=Fk*Pk*Fk' + Q;
    
    % Observation
    N = [sqrt(rx)*randn(1,1); sqrt(rv)*randn(1,1); sqrt(r)*randn(10,1)];
    z(:,k+1)= H*[x(k+1) v(k+1) p']' + N; % Measure at the correct position x(k+1) v(k+1) and p
    vv(:,k+1)=z(:,k+1)-zhat(:,k+1);  % Innovation vector, i.e. discrepancy between measures
    S = H*Pk*H' + R; 
    
    % update
    W = Pk * H' * inv(S); 
    xhat(:,k+1)=xhat(:,k+1)+W*vv(:,k+1);
    Pk = Pk - W*H*Pk;
end;

figure; title('1D SLAM with KF');
ax=[1 NumberTimeStamps MapDimension(1) MapDimension(2)];
axis(ax); hold on;

for i = 1:NumberTimeStamps
   plot(i,x(1,i),'g');
   plot(i,xhat(1,i),'r');
   for j=1:NumberLandmarks
      plot(i,xhat(2+j,i),'b');
      plot(i,z(2+j,i),'m');
      plot(i,p(j),'k');          
   end 
end
