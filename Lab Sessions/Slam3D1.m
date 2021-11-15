% 3D SLAM with linear KF - Moving vehicle - Relative measurement - Limited
% Sensor Range - Observing (x,v,landmarks) - Landmarks updated once
% observed
%
% A 3 DOF underwater robot is moving along a path detecting some
% motionless landmarks; the position/velocity of the robot and position
% of landmarks are computed by means of Simultaneos Localization and 
% Mapping using a linear Kalman Filter.  
% All the measures are relative to robot position.
% The measuring sensor is limited to a certain range distance.
% Landmarks are updated in the state once observed.
%
% Programmed by Joaquim Salvi

clear all;
NumberTimeStamps = 600;
MapDimension = [1,200;1,150;-100,0];   % X Y Z
MaximumRange = 50;

% INTRODUCE LANDMARKS
figure(1); clf;
title('Introduce 3D Landmarks');
v=[MapDimension(1,1)-10 MapDimension(1,2)+20 MapDimension(2,1)-10 MapDimension(2,2)+10];
axis(v); hold on;
xlabel('x'); ylabel('y');
text((MapDimension(1,2))/2,MapDimension(2,2)+5,'X-Y Plane');
text(MapDimension(1,2)+20-20,MapDimension(2,2)+5,'Z Plane');
plot([MapDimension(1,1); MapDimension(1,1)],[MapDimension(2,1); MapDimension(2,2)],'k-');
plot([MapDimension(1,1); MapDimension(1,2)],[MapDimension(2,1); MapDimension(2,1)],'k-');
plot([MapDimension(1,1); MapDimension(1,2)],[MapDimension(2,2); MapDimension(2,2)],'k-');
plot([MapDimension(1,2); MapDimension(1,2)],[MapDimension(2,1); MapDimension(2,2)],'k-');
plot([MapDimension(1,2); MapDimension(1,2)+10],[MapDimension(2,1); MapDimension(2,1)],'k-');
plot([MapDimension(1,2); MapDimension(1,2)+10],[MapDimension(2,2); MapDimension(2,2)],'k-');
plot([MapDimension(1,2)+10; MapDimension(1,2)+10],[MapDimension(2,1); MapDimension(2,2)],'k-');
stepz = (MapDimension(3,2)-MapDimension(3,1))/10;
stepy = (MapDimension(2,2)-MapDimension(2,1))/10; posy = MapDimension(2,1);
for i=MapDimension(3,1):stepz:MapDimension(3,2)
    st = sprintf('%0d',i);
    text(MapDimension(1,2)+11,posy,st);
    posy = posy + stepy;
end
pin=1; button = 0;
nlandmarks=0; z = 0;
poszx = MapDimension(1,2)+5;
poszy = MapDimension(2,2);
plot(poszx,poszy,'ro');
% Get input landmarks graphically, click right button to finish
while button ~= 3
   [x,y,button]=ginput(1);
   pin= ~isempty(x);
   if pin && button ~= 3
       if x > MapDimension(1,1) && x < MapDimension(1,2) && y > MapDimension(2,1) && y < MapDimension(2,2) 
          nlandmarks = nlandmarks+1;
          plot(x,y,'r*'); st = sprintf('%0d',round(z)); text(x+2,y+2,st);
          pp(:,nlandmarks)=[x;y;z];
       elseif x>MapDimension(1,2) && x < MapDimension(1,2)+10 && y > MapDimension(2,1) && y < MapDimension(2,2) 
           plot(poszx,poszy,'wo');
           z = MapDimension(3,1) + y*(MapDimension(3,2)-MapDimension(3,1))/(MapDimension(2,2)-MapDimension(2,1));
           poszy = y;
           plot(poszx,poszy,'ro');
       end
   end
end
hold off;
NumberLandmarks = size(pp,2);

% DEFINE TRAJECTORY
% Definition of the real trajectory; vehicle with constant velocity
figure(2); clf;
title('Introduce trajectory around landmarks');
v=[MapDimension(1,1)-10 MapDimension(1,2)+20 MapDimension(2,1)-10 MapDimension(2,2)+10];
axis(v); hold on;
xlabel('x'); ylabel('y');
text((MapDimension(1,2))/2,MapDimension(2,2)+5,'X-Y Plane');
text(MapDimension(1,2)+20-20,MapDimension(2,2)+5,'Z Plane');
plot([MapDimension(1,1); MapDimension(1,1)],[MapDimension(2,1); MapDimension(2,2)],'k-');
plot([MapDimension(1,1); MapDimension(1,2)],[MapDimension(2,1); MapDimension(2,1)],'k-');
plot([MapDimension(1,1); MapDimension(1,2)],[MapDimension(2,2); MapDimension(2,2)],'k-');
plot([MapDimension(1,2); MapDimension(1,2)],[MapDimension(2,1); MapDimension(2,2)],'k-');
plot([MapDimension(1,2); MapDimension(1,2)+10],[MapDimension(2,1); MapDimension(2,1)],'k-');
plot([MapDimension(1,2); MapDimension(1,2)+10],[MapDimension(2,2); MapDimension(2,2)],'k-');
plot([MapDimension(1,2)+10; MapDimension(1,2)+10],[MapDimension(2,1); MapDimension(2,2)],'k-');
stepz = (MapDimension(3,2)-MapDimension(3,1))/10;
stepy = (MapDimension(2,2)-MapDimension(2,1))/10; posy = MapDimension(2,1);
for i=MapDimension(3,1):stepz:MapDimension(3,2)
    st = sprintf('%0d',i);
    text(MapDimension(1,2)+11,posy,st);
    posy = posy + stepy;
end
for i = 1:NumberLandmarks
    plot(pp(1,i),pp(2,i),'r*'); 
    st = sprintf('%0d',round(pp(3,i))); text(pp(1,i)+2,pp(2,i)+2,st);
end;
pin = 1; button = 0; fi = 0;
npoints = 0; dist = 0; z = 0;
poszx = MapDimension(1,2)+5;
poszy = MapDimension(2,2);
plot(poszx,poszy,'ro');
% get trajectory segments, click right button to finish 
while button ~= 3
   [x,y,button]=ginput(1);
   pin= ~isempty(x);
   if pin && button ~= 3
       if x > MapDimension(1,1) && x < MapDimension(1,2) && y > MapDimension(2,1) && y < MapDimension(2,2) 
          npoints = npoints + 1;
          t(:,npoints)=[x;y;z];
          plot(x,y,'go'); st = sprintf('%0d',round(z)); text(x+2,y+2,st);
          if npoints > 1
            plot(t(1,npoints-1:npoints),t(2,npoints-1:npoints),'g-');
            dist = dist + norm(t(:,npoints) - t(:,npoints-1));
          end
       elseif x>MapDimension(1,2) && x < MapDimension(1,2)+10 && y > MapDimension(2,1) && y < MapDimension(2,2) 
           plot(poszx,poszy,'wo');
           z = MapDimension(3,1) + y*(MapDimension(3,2)-MapDimension(3,1))/(MapDimension(2,2)-MapDimension(2,1));
           poszy = y;
           plot(poszx,poszy,'ro');
       end
   end
end
% Sampling NumberTimeStamps points in the given trajectory.
point = 2; dist2=0; incdist=dist/NumberTimeStamps;
tt(:,1)=t(:,1);
for i = 2:NumberTimeStamps
    tt(:,i)=tt(:,i-1)+ incdist*((t(:,point)-t(:,point-1))/norm(t(:,point)-t(:,point-1))); % tx,ty,tz trajectories
    vv(:,i-1)=tt(:,i)-tt(:,i-1); % vx,vy,vz velocities
    plot(tt(1,i),tt(2,i),'b.');
    dist2 = dist2 + incdist;
    if (dist2 + incdist) > norm(t(:,point)-t(:,point-1)) && abs((dist2 + incdist)-norm(t(:,point)-t(:,point-1))) > abs(dist2-norm(t(:,point)-t(:,point-1)))
        point = point + 1; dist2 = 0;
    end
end
plot(tt(1,:),tt(2,:),'b.');
hold off;

% DEFINE INITIAL PARAMETERS
v = vv;  % Velocity is the discrepancy between trajectory points
x = tt;  % Trajectory is the interpolated trajectory introduced by user
         % Landmarks are arranged in a vector form (x1,y1,x2,y2,... yn)
clear vv;
clear z;
for i=1:NumberLandmarks
    p(3*i-2)=pp(1,i);
    p(3*i-1)=pp(2,i);
    p(3*i)=pp(3,i);
end     

% Definition of process noise and initial noises
ri = (30*0.5)^2;  % landmark measurement noise, change to see effect
rxi = (3*0.5)^2; % position measurement noise, change to see effect
rvi = (1*0.5)^2; % velocity measurement noise, change to see effect
pn = (10*0.1)^2; % process noise, change to see effect

% Definition of Measurement matrix to use in prediction
% Landmark distance are relative to robot position
HP = diag(ones(3+3,1));

% Definition of Measurement matrix to use in ground truth
% Landmark distance are relative to robot position
HO = [diag(ones(3+3,1)) zeros(3+3,3*NumberLandmarks)];
for i=1:NumberLandmarks
    HO = [HO;-1 0 0 0 0 0 zeros(1,3*(i-1)) 1 0 0 zeros(1,3*(NumberLandmarks-i))];
    HO = [HO;0 -1 0 0 0 0 zeros(1,3*(i-1)) 0 1 0 zeros(1,3*(NumberLandmarks-i))];
    HO = [HO;0 0 -1 0 0 0 zeros(1,3*(i-1)) 0 0 1 zeros(1,3*(NumberLandmarks-i))];    
end

% Definition of the State matrix
% The State matrix is a diagonal of ones, which means that the prediction
% of next state is equivalent to current state, except the position which
% is updated with the current velocity at every time stamp.
Fk = diag(ones(1,3+3)); Fk(1,4)=1; Fk(2,5)=1; Fk(3,6)=1;   

% Definition of the Control vector
% The control vector is null, which means that the are no external inputs
% that change the robot state.
Uk = zeros(3+3,1);  

% Definition of the Covariance matrix
% The initial covariance matrix is defined with a large diagonal uncertainity
% in both state of the robot and position of the landmarks and with an
% equivalent uncertainity which means that none prevail over the other.
Pk = [(rxi+rvi) 0 0 rvi 0 0;0 (rxi+rvi) 0 0 rvi 0; 0 0 (rxi+rvi) 0 0 rvi; rvi 0 0 rvi 0 0 ;0 rvi 0 0 rvi 0;0 0 rvi 0 0 rvi];

%Definition of the Process Noise Matrix
Q = diag(pn*ones(1,3+3));

% Definition of measurement noises (may differ from initial noises
r = (10*0.5)^2;  % landmark measurement noise, change to see effect
rx = (2*0.5)^2; % position measurement noise, change to see effect
rv = (0.1*0.5)^2; % velocity measurement noise, change to see effect

%Definition of the Measurement Noise Matrix
R = diag([rx*ones(1,3) rv*ones(1,3)]); 

% Initial prediction of the vector state
for kk = 1:NumberTimeStamps
    xhat{kk} = [];
    zhat{kk} = [];
    Un{kk} = [];
end
xhat{1}= [x(:,1)' v(:,1)']';
N = sqrt(pn)*randn(3+3,1); xhat{1}(1:6) = xhat{1}(1:6) + N; % adding process noise
xtt(:,1) = xhat{1}(1:3);  % Only for plotting purposes


% KALMAN FILTER LOOP
figure(3); clf; 
handle = gcf;
set(handle,'DoubleBuffer','on'); 
view(3);

landobs = []; % Set of landmarks observed at least once, i.e. already updated in Pk.
plotland(:,1) = zeros(NumberLandmarks,1);
for k = 1:NumberTimeStamps-2
    
    % Prediction
    xhat{k+1} = Fk*xhat{k}+Uk;     % Prediction of next state
    zhat{k+1} = HP*xhat{k+1};      % Measure at the predicted position
    Pk = Fk*Pk*Fk' + Q;
    
    % Observation
    N = [sqrt(rx)*randn(3,1); sqrt(rv)*randn(3,1); sqrt(r)*randn(3*NumberLandmarks,1)];
    z(:,k+1)= HO*[x(:,k+1)' v(:,k+1)' p]' + N; % Measure at the correct position x(k+1) v(k+1) and p
    vvt=z(1:6,k+1)-zhat{k+1}(1:6);  % Innovation vector, i.e. discrepancy between measures
    znot(:,k+1)= HO*[x(:,k+1)' v(:,k+1)' p]'; % Measure at the correct position x(k+1) v(k+1) and p
    
    landdec = []; landnotdec = []; % landmarks detected/non-detected sets
    plotland(:,k+1) = zeros(NumberLandmarks,1); % only for plotting purposes
    for kk=7:3:7+3*NumberLandmarks-1  % Explore the detection of landmarks
        if norm(znot(kk:kk+2,k+1)) < MaximumRange % Landmark in not-noisy range?       
            landdec = [landdec; (kk-1)/3-1]; % Add detected landmark identifier to the set
            [check,pos] = ismember((kk-1)/3-1,landobs); % Check if landmark was observed in the past
            if check
                vvt=[vvt;z(kk:kk+2,k+1)-zhat{k+1}(6+3*pos-2:6+3*pos)];  % Innovation vector, i.e. discrepancy between measures
                plotland((kk-1)/3-1,k+1) = 1;
            else
                landnotdec = [landnotdec; (kk-1)/3-1]; % Add landmark as non-detected for another step
            end
        else
            landnotdec = [landnotdec; (kk-1)/3-1]; % Add non-detected landmark identifier to the set
        end
    end
    
    Ht = HP; 
    for kk=size(landobs,1):-1:1  % Removing rows/column of once observed but now non-detected landmarks
        [check,pos] = ismember(landobs(kk),landnotdec);
        if check
            Ht(6+3*kk-2:6+3*kk,:) = []; 
        end
    end

    Rt = diag([rx*ones(1,3) rv*ones(1,3) r*ones(1,size(Ht,1)-6)]);
    S = Ht*Pk*Ht' + Rt;
    
    % Update
    W = Pk * Ht' * inv(S); 
    xhat{k+1} = xhat{k+1}+W*vvt;
    Pk = Pk - W*Ht*Pk;

    Un{k+1}=diag(Pk);
    xtt(:,k+1) = xhat{k+1}(1:3);
    
    % Plotting results
    clf; hold on;
    title('3D SLAM with KF');
    mapdim=[MapDimension(1,1) MapDimension(1,2) MapDimension(2,1) MapDimension(2,2) MapDimension(3,1) MapDimension(3,2)];
    axis(mapdim); xlabel('x'); ylabel('y'); zlabel('z');
    plot3(x(1,1:k),x(2,1:k),x(3,1:k),'g');
    plot3(xtt(1,1:k),xtt(2,1:k),xtt(3,1:k),'r');
    plot3(x(1,k+1),x(2,k+1),x(3,k+1),'g.');
    plot3(xhat{k+1}(1),xhat{k+1}(2),xhat{k+1}(3),'r.');
    [xe,ye,ze]=ellipsoid(xhat{k+1}(1),xhat{k+1}(2),xhat{k+1}(3),3*sqrt(Un{k+1}(1)),3*sqrt(Un{k+1}(2)),3*sqrt(Un{k+1}(3)));
    plot3(xe,ye,ze,'r');
    plot3(pp(1,:),pp(2,:),pp(3,:),'r*');
    for j=7:3:size(xhat{k+1},1)-1
        plot3(xhat{k+1}(j),xhat{k+1}(j+1),xhat{k+1}(j+2),'b.');
        [xe,ye,ze]=ellipsoid(xhat{k+1}(j),xhat{k+1}(j+1),xhat{k+1}(j+2),3*sqrt(Un{k+1}(j)),3*sqrt(Un{k+1}(j+1)),3*sqrt(Un{k+1}(j+2)));
        plot3(xe,ye,ze,'b');
    end
    drawnow;
    hold off;

    % Adding detected landmarks to the state
    for kk=1:size(landdec,1)
        if ~ismember(landdec(kk),landobs)
            landobs = [landobs; landdec(kk)];
            landobs = sort(landobs,'ascend');
            [check,pos] = ismember(landdec(kk),landobs);
            if pos == size(landobs,1)
                Fk = [Fk zeros(size(Fk,1),3)]; Fk = [Fk; [zeros(3,size(Fk,2)-3) [1 0 0; 0 1 0; 0 0 1]]];
                Q = [Q zeros(size(Q,1),3)]; Q = [Q; [zeros(3,size(Q,2)-3) [0 0 0; 0 0 0; 0 0 0]]];           % [pn 0 0; 0 pn 0; 0 0 pn]]];
                Pk = [Pk Pk(1:size(Pk,1),1:3)]; Pk = [Pk; [Pk(1:3,1:size(Pk,2)-3) [Pk(1,1)+ri 0 0; 0 Pk(1,1)+ri 0; 0 0 Pk(1,1)+ri]]];
%                xhat{k+1} = [xhat{k+1}; xhat{k+1}(1:3) + z(7+3*(landdec(kk)-1):7+3*(landdec(kk)-1)+2,k+1)];
                xhat{k+1} = [xhat{k+1}; p(3*landdec(kk)-2:3*landdec(kk))']; 
                Uk = [Uk; zeros(3,1)];
                HP = [HP zeros(3,size(HP,1))']; HP = [HP; [[-1 0 0; 0 -1 0; 0 0 -1] zeros(3,size(HP,2)-6) [1 0 0; 0 1 0; 0 0 1]]];
            else
                Fk = [Fk(:,1:6+3*(pos-1)) zeros(size(Fk,1),3) Fk(:,6+3*(pos-1)+1:size(Fk,2))]; 
                Fk = [Fk(1:6+3*(pos-1),:); zeros(3,size(Fk,2)); Fk(6+3*(pos-1)+1:size(Fk,1),:)];
                Fk(6+3*pos-2,6+3*pos-2) = 1; Fk(6+3*pos-1,6+3*pos-1) = 1; Fk(6+3*pos,6+3*pos) = 1;
                Q = [Q(:,1:6+3*(pos-1)) zeros(size(Q,1),3) Q(:,6+3*(pos-1)+1:size(Q,2))]; 
                Q = [Q(1:6+3*(pos-1),:); zeros(3,size(Q,2)); Q(6+3*(pos-1)+1:size(Q,1),:)];
                Q(6+3*pos-2,6+3*pos-2) = 0; Q(6+3*pos-1,6+3*pos-1) = 0; Q(6+3*pos,6+3*pos) = 0;
%                 Q(6+3*pos-2,6+3*pos-2) = pn; Q(6+3*pos-1,6+3*pos-1) = pn; Q(6+3*pos,6+3*pos) = pn;
                Pk = [Pk(:,1:6+3*(pos-1)) Pk(1:size(Pk,1),1:3) Pk(:,6+3*(pos-1)+1:size(Pk,2))]; 
                Pk = [Pk(1:6+3*(pos-1),:); Pk(1:3,1:size(Pk,2)); Pk(6+3*(pos-1)+1:size(Pk,1),:)];
                Pk(6+3*pos-2,6+3*pos-2) = Pk(1,1)+ri; Pk(6+3*pos-1,6+3*pos-1) = Pk(2,2)+ri; Pk(6+3*pos,6+3*pos) = Pk(3,3)+ri;
%                xhat{k+1} = [xhat{k+1}(1:6+3*(pos-1)); xhat{k+1}(1:3) + z(7+3*(landdec(kk)-1):7+3*(landdec(kk)-1)+2,k+1); xhat{k+1}(6+3*(pos-1)+1:size(xhat{k+1},1))];
                xhat{k+1} = [xhat{k+1}(1:6+3*(pos-1)); p(3*landdec(kk)-2:3*landdec(kk))'; xhat{k+1}(6+3*(pos-1)+1:size(xhat{k+1},1))];
                Uk = [Uk(1:6+3*(pos-1)); zeros(3,1); Uk(6+3*(pos-1)+1:size(Uk,1))];
                HP = [HP(:,1:6+3*(pos-1)) zeros(size(HP,1),3) HP(:,6+3*(pos-1)+1:size(HP,2))]; 
                HP = [HP(1:6+3*(pos-1),:); [[-1 0 0; 0 -1 0; 0 0 -1] zeros(3,3*pos) [1 0 0; 0 1 0; 0 0 1] zeros(3,size(HP,2)-6-3*pos)]; HP(6+3*(pos-1)+1:size(HP,1),:)];
            end
        end
    end
    
end;

% PLOTTING RESULTS
figure(4); clf; title('3D SLAM with KF'); view(3);
mapdim=[MapDimension(1,1) MapDimension(1,2) MapDimension(2,1) MapDimension(2,2) MapDimension(3,1) MapDimension(3,2)];
axis(mapdim); hold on; xlabel('x'); ylabel('y'); zlabel('z');
for i = 2:NumberTimeStamps-2
   plot3(x(1,i),x(2,i),x(3,i),'g.');
   plot3(xhat{i}(1),xhat{i}(2),xhat{i}(3),'r.');
   [xe,ye,ze]=ellipsoid(xhat{i}(1),xhat{i}(2),xhat{i}(3),3*sqrt(Un{i}(1)),3*sqrt(Un{i}(2)),3*sqrt(Un{i}(3)));
   plot3(xe,ye,ze,'r');
   kk = 1;
   for j=1:NumberLandmarks
       if max(plotland(j,1:i)) == 1
           plot3(xhat{i}(1)+zhat{i}(6+kk),xhat{i}(2)+zhat{i}(6+kk+1),xhat{i}(3)+zhat{i}(6+kk+2),'m.');
           plot3(xhat{i}(6+kk),xhat{i}(6+kk+1),xhat{i}(6+kk+2),'b.'); 
           if mod(i,10) == 2
               [xe,ye,ze]=ellipsoid(xhat{i}(6+kk),xhat{i}(6+kk+1),xhat{i}(6+kk+2),3*sqrt(Un{i}(6+kk)),3*sqrt(Un{i}(6+kk+1)),3*sqrt(Un{i}(6+kk+2)));
               plot3(xe,ye,ze,'b');
           end
           kk = kk + 3;
       end
   end
end
kk = 1;
for j=1:NumberLandmarks
    plot3(pp(1,j),pp(2,j),pp(3,j),'r*');
    st=sprintf('%d',j); 
    text(pp(1,j)+2,pp(2,j)+2,pp(3,j)+2,st);
    if max(plotland(j,1:NumberTimeStamps-2)) == 1
        plot3(xhat{NumberTimeStamps-2}(6+kk),xhat{NumberTimeStamps-2}(6+kk+1),xhat{NumberTimeStamps-2}(6+kk+2),'go');
        kk = kk + 3;
    end
end
hold off; zoom on;