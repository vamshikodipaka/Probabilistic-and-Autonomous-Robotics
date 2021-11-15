% 2D SLAM with linear KF - Moving vehicle - Relative measurement -
% Unlimited Sensor Range - Observing (x,v,landmarks)
%
% A 2 DOF mobile robot is moving along a path detecting some
% motionless landmarks; the position/velocity of the robot and position
% of landmarks are computed by means of Simultaneos Localization and 
% Mapping using a linear Kalman Filter.  
% All the measures are relative to robot position.
% The measuring range sensor is unlimited.
%
% Programmed by Joaquim Salvi

clear all;
NumberTimeStamps = 2000;
MapDimension = [1,100;1,100];

% INTRODUCE LANDMARKS
figure(1); clf; title('Introduce Landmarks');
v=[MapDimension(1,1) MapDimension(1,2) MapDimension(2,1) MapDimension(2,2)];
axis(v); hold on;
pin=1; button = 0;
nlandmarks=0;
% Get input landmarks graphically, click right button to finish
while button ~= 3
   [x,y,button]=ginput(1);
   pin= ~isempty(x);
   if pin && button ~= 3
      nlandmarks = nlandmarks+1;
      plot(x,y,'r*')
      pp(:,nlandmarks)=[x;y];
   end
end
hold off;
NumberLandmarks = size(pp,2);

% DEFINE TRAJECTORY
% Definition of the real trajectory; vehicle with constant velocity
figure(2); clf;
title('Introduce trajectory around landmarks');
v=[MapDimension(1,1) MapDimension(1,2) MapDimension(2,1) MapDimension(2,2)];
axis(v); hold on;
plot(pp(1,:),pp(2,:),'r*');
pin = 1; button = 0; fi = 0;
npoints = 0; dist = 0;
% get trajectory segments, click right button to finish 
while button ~= 3
   [x,y,button]=ginput(1);
   pin= ~isempty(x);
   if pin && button ~= 3
      npoints = npoints + 1;
      t(:,npoints)=[x;y];
      plot(x,y,'go');
      if npoints > 1
        plot(t(1,npoints-1:npoints),t(2,npoints-1:npoints),'g-');
        dist = dist + norm(t(:,npoints) - t(:,npoints-1));
      end
   end
end
% Sampling NumberTimeStamps points in the given trajectory.
point = 2; dist2=0; incdist=dist/NumberTimeStamps;
tt(:,1)=t(:,1);
for i = 2:NumberTimeStamps
    tt(:,i)=tt(:,i-1)+ incdist*((t(:,point)-t(:,point-1))/norm(t(:,point)-t(:,point-1))); % tx,ty trajectories
    vv(:,i-1)=tt(:,i)-tt(:,i-1); % vx,vy velocities
    plot(tt(1,i),tt(2,i),'b.');
    dist2 = dist2 + incdist;
    if (dist2 + incdist) > norm(t(:,point)-t(:,point-1)) && abs((dist2 + incdist)-norm(t(:,point)-t(:,point-1))) > abs(dist2-norm(t(:,point)-t(:,point-1)))
        point = point + 1; dist2 = 0;
    end
end
plot(tt(1,:),tt(2,:),'b.');
hold off;

% DEFINE INITIAL PARAMETERS
v = vv;  % Velocity norm is constant and equal to 1, but not vx, vy
x = tt;  % Trajectory is the interpolated trajectory introduced by user
         % Landmarks are arranged in a vector form (x1,y1,x2,y2,... yn)
clear vv;
for i=1:nlandmarks
    p(2*i-1)=pp(1,i);
    p(2*i)=pp(2,i);
end     

% Definition of process and measurement noises
r = (30*0.5)^2;  % landmark measurement noise, change to see effect
rx = (3*0.5)^2; % position measurement noise, change to see effect
rv = (0.1*0.5)^2; % velocity measurement noise, change to see effect
pn = (0.001)^2; % process noise, change to see effect

% Definition of Measurement matrix
% Landmark distance are relative to robot position
H = [diag(ones(2+2,1)) zeros(2+2,2*NumberLandmarks)];
for i=1:2:2*NumberLandmarks
    H = [H;-1 0 0 0 zeros(1,i-1) 1 zeros(1,2*NumberLandmarks-i)];
    H = [H;0 -1 0 0 zeros(1,i) 1 zeros(1,2*NumberLandmarks-i-1)];
end

% Definition of the State matrix
% The State matrix is a diagonal of ones, which means that the prediction
% of next state is equivalent to current state, except the position which
% is updated with the current velocity at every time stamp.
Fk = diag(ones(1,2+2+2*NumberLandmarks)); Fk(1,3)=1; Fk(2,4)=1;   

% Definition of the Control vector
% The control vector is null, which means that the are no external inputs
% that change the robot state.
Uk = zeros(2+2+2*NumberLandmarks,1);  

% Definition of the Covariance matrix
% The initial covariance matrix is defined with a large diagonal uncertainity
% in both state of the robot and position of the landmarks and with an
% equivalent uncertainity which means that none prevail over the other.
Pk = diag(ones(1,2+2+2*NumberLandmarks)); Pki=[];
for i = 1:2+NumberLandmarks
    Pki = [Pki [1 0;0 1]];
end
Pk(1:2,:) = Pki; 
Pk(:,1:2) = Pki';
Pk = r*Pk;
Pk(1,1)=rx+rv; Pk(2,2)=rx+rv; Pk(3,3)=rv; Pk(4,4)=rv;
Pk(1,3)=rv; Pk(2,4)=rv; Pk(3,1)=rv; Pk(4,2)=rv;
for i = 5:5+2*NumberLandmarks-1
    Pk(i,i) = r+rx;
end

%Definition of the Process/Measure variance; landmarks are motionless so
%they are not influenced by noise.
Q = diag([pn*ones(1,2+2) 0*pn*ones(1,2*NumberLandmarks)]); 
%Definition of the Measurement Noise Matrix
R = diag([rx*ones(1,2) rv*ones(1,2) r*ones(1,2*NumberLandmarks)]); 

% Initial prediction of the vector state
xhat(:,1)= [x(:,1)' v(:,1)' p]';
N = sqrt(pn)*randn(2+2,1); xhat(1:4,1) = xhat(1:4,1) + N; % adding process noise

% KALMAN FILTER LOOP
figure(3); clf; 
handle = gcf;
set(handle,'DoubleBuffer','on')

for k = 1:NumberTimeStamps-2
    
    % Prediction
    xhat(:,k+1)=Fk*xhat(:,k)+Uk;     % Prediction of next state
    zhat(:,k+1)= H*xhat(:,k+1);      % Measure at the predicted position
    Pk=Fk*Pk*Fk' + Q;
    
    % Observation
    N = [sqrt(rx)*randn(2,1); sqrt(rv)*randn(2,1); sqrt(r)*randn(2*NumberLandmarks,1)];
    z(:,k+1)= H*[x(:,k+1)' v(:,k+1)' p]' + N; % Measure at the correct position x(k+1) v(k+1) and p
    vv(:,k+1)=z(:,k+1)-zhat(:,k+1);  % Innovation vector, i.e. discrepancy between measures
    S = H*Pk*H' + R; 
    
    % Update
    W = Pk * H' * inv(S); 
    xhat(:,k+1)=xhat(:,k+1)+W*vv(:,k+1);
    Pk = Pk - W*H*Pk;
    Un(:,k+1)=diag(Pk);  % The Uncertainity is stored for plotting purposes

    % Plotting results
    clf; hold on;
    title('2D SLAM with KF');
    mapdim=[MapDimension(1,1) MapDimension(1,2) MapDimension(2,1) MapDimension(2,2)];
    axis(mapdim); 
    plot(x(1,1:k),x(2,1:k),'g');
    plot(xhat(1,1:k),xhat(2,1:k),'r');
    plot(x(1,k+1),x(2,k+1),'g.');
    plot(xhat(1,k+1),xhat(2,k+1),'r.');
    [xe,ye,ze]=ellipsoid(xhat(1,k+1),xhat(2,k+1),0,3*sqrt(Un(1,k+1)),3*sqrt(Un(2,k+1)),0);
    plot(xe(floor(size(xe,2)/2),:),ye(floor(size(ye,2)/2),:));
    plot(pp(1,:),pp(2,:),'r*');
    for j=5:2:5+2*NumberLandmarks-1
%        plot(xhat(1,1:k+1)+zhat(j,1:k+1),xhat(2,1:k+1)+zhat(j+1,1:k+1),'m.');
        plot(xhat(j,k+1),xhat(j+1,k+1),'b.');
        [xe,ye,ze]=ellipsoid(xhat(j,k+1),xhat(j+1,k+1),0,3*sqrt(Un(j,k+1)),3*sqrt(Un(j+1,k+1)),0);
        plot(xe(floor(size(xe,2)/2),:),ye(floor(size(ye,2)/2),:));
    end
    drawnow;
    hold off;
   
end;

% PLOTTING RESULTS
figure(4); clf; title('2D SLAM with KF');
mapdim=[MapDimension(1,1) MapDimension(1,2) MapDimension(2,1) MapDimension(2,2)];
axis(mapdim); hold on;
for i = 1:NumberTimeStamps-2
   plot(x(1,i),x(2,i),'g.');
   plot(xhat(1,i),xhat(2,i),'r.');
   [xe,ye,ze]=ellipsoid(xhat(1,i),xhat(2,i),0,2*sqrt(Un(1,i)),2*sqrt(Un(2,i)),0);
   plot(xe(floor(size(xe,2)/2),:),ye(floor(size(ye,2)/2),:));
   for j=5:2:5+2*NumberLandmarks-1
     plot(xhat(1,i)+zhat(j,i),xhat(2,i)+zhat(j+1,i),'m.');
     plot(xhat(j,i),xhat(j+1,i),'b.');
     if mod(i,10) == 2
         [xe,ye,ze]=ellipsoid(xhat(j,i),xhat(j+1,i),0,2*sqrt(Un(j,i)),2*sqrt(Un(j+1,i)),0);
         plot(xe(floor(size(xe,2)/2),:),ye(floor(size(ye,2)/2),:));
     end
   end
end
plot(pp(1,:),pp(2,:),'r*');
for j=5:2:5+2*NumberLandmarks-1
    plot(xhat(j,NumberTimeStamps-2),xhat(j+1,NumberTimeStamps-2),'go');
end
hold off; zoom on;

