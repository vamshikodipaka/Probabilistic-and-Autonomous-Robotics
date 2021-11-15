clc
close all;
clear all;
%%
data=load('observations.txt');
z = data;
states= load('states.txt')
T = 0.1;
mk = 10;
nk = 0.02;
A=[1 T; 0 1];

B= [T^2/2; T];
C=[1 0];
wk = 0.2* [T^2/2; T];
Q = wk' * wk;
R=100;
u=1;

x0=[0; 0];
P0=[10000 0; 0 10000];
xhat=discrete_kalman_filter_vn( A, B, C, Q, R, z, u, x0, P0 );
xhat = xhat';
size(xhat);
hold on;
plot(z(1:200),'-b');
hold on;

plot(xhat(1:200),'-r');
hold on;

plot(states(1:200),'-g');
hold on;

