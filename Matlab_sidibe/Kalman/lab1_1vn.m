clc; clear all;

z = load('./measurements.txt');

plot(z,'xr');
xlabel('no of measurements');
ylabel('measurement value');

true_value = 0.512 * ones(1,length(z));

%std.dev of measurements noise
R = std(z - 0.512);

u = 1;
A = 1;
B = 0;
C = 1;
Q = 1e-4;
% u = zeros(1,length(z));
u = 0;
x0 = 0.5;
p0 = 1;

% [ x_hat ] = discrete_kalman_filter( A, B, C, Q, R, z, u, x0, p0 );
[ x_hat, P_hat ] = discrete_kf( A, B, C, Q, R, z, u, x0, p0 );

i = 1:length(z);
plot(i,z,'xb',i,x_hat,'r',i,true_value,'b')
legend('measurements','predicted values','true value');

figure
plot(P_hat);