clc
clear all
close all
format long;

% Laod data.mat file
load('data.mat');

%% Q2: Finding instrinsic params by Kruppas method

%% Q2.1 Finding instrinsic params by Kruppas Classical Method
p0 = [A(1,:) A(2,2:3)]; % Reading required initial values 

OPTIONS = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt','TolFun', 1e-32,'TolX',1e-32); 
% in above step I choose optimiser options

% Using the optimiser non-linear least square to minimise cost func
output_Classic = lsqnonlin(@(p0) costFuncKClassical(Fs,p0),p0,[],[],OPTIONS);

% -- Find the costFuncKClassical below --------------

% Display optimidsed values of intrinsics
KClassic = [output_Classic(1:3); 0 output_Classic(4) output_Classic(5); 0 0 1];

disp('------- Estimated Intrinsics by Classical Kruppas method');
disp(KClassic)


%% Q2.2 Finding instrinsic params by Kruppas Simplified method

% Using the optimiser non-linear least square to minimise cost func
output_Simplif = lsqnonlin(@(p0) costFuncKSimplified(Fs,p0),p0,[],[],OPTIONS); %set optimiser options

% --Find the costFuncKSimplified below --------------

% Display optimidsed values of intrinsics
KSimplif = [output_Simplif(1:3); 0 output_Simplif(4) output_Simplif(5); 0 0 1];

disp('----- Estimated Intrinsicss by Simplified Kruppas method');
disp(KSimplif)


%% Q1: Mendonça-Cipolla autocalibration method ---------

% Using the optimiser non-linear least square to minimise cost func
outputMCipolla = lsqnonlin(@(p0) costFuncMenCipolla(Fs,p0),p0,[],[],OPTIONS);
% in above step I choose optimiser options

% --Find the costFuncMenCipolla below --------------

% Display optimidsed values of intrinsics
K_MCipolla = [outputMCipolla(1:3); 0 outputMCipolla(4) outputMCipolla(5); 0 0 1];

disp('------ Estimated Intrinsics by Mendonça-Cipollas method');
disp(K_MCipolla)

%% Q3 : Dual Absolute Quadric ----------
% Before proceeding - find plane at infinity from MQM' ~ w (from DAQ)

% finding normal to the plane at infinity
findnormal = getNormalToPlaneAtInf(PPM,A);
findnormal = findnormal(1:3)';
disp('---- Estimated normal to plane at infinity by DAQ');
disp(findnormal)

%find HInf = [e21]F+e21*n'
HInfi = computeHInf(Fs,findnormal); 

p0 = [A(1,:) A(2,2:3)]; % initial value
OPTIONS = optimoptions('lsqnonlin', 'Algorithm','levenberg-marquardt','TolFun', 1e-100,'TolX',1e-100, 'MaxIter', 1000); %set optimiser options
w_DAQ = lsqnonlin(@(n)  costFuncW( HInfi, p0),p0,[],[],OPTIONS);
K_DAC = [w_DAQ(1:3); 0 w_DAQ(4) w_DAQ(5); 0 0 1];

disp('---- Estimated Intrinsics by DAQ');
disp(K_DAC)

%% HERE BELOW YOU WILL FIND THE FUNCTNS REQUIRED ------------

%% --------- 1. COST FUNC TO MENCIPOLLA's EQU -----------------

function errestim = costFuncMenCipolla(Fs, p)
% Fs - given fundamental matrix ---------------
% initial intrinsic-values

A = [p(1) p(2) p(3); 0 p(4) p(5); 0 0 1];
w = A * A';

% ----
errestim = [];

for i = 1 : size(Fs,3) - 1
    for j = i+1 : size(Fs,3)
        
        % calculating essential matrix 
        Emat = A' * Fs(:,:,i,j) * A;
        
        % calculating SVD here
        [~,Z,~] = svd(Emat);
        
        % taking the sigma values of N (decomposed svd) 
        x1 = Z(1,1);        x2 = Z(2,2);
        
        % estimating cost func ....
        differrence = 1/45 * (x1 - x2)/x2;
        
        errestim = [errestim differrence];
        
    end
end
        
end


%% ------------2.1 COST FUNC TO CLASSICAL KRUPPA'S EQU--------------------

function errestim = costFuncKClassical(Fs, p)
% Fs - given fundamental matrix ---------------
% initial intrinsic-values

A=[p(1) p(2) p(3); 0 p(4) p(5); 0 0 1];

w = A * A';
%---
errestim = [];

for i = 1 : size(Fs,3)-1
    for j = i+1 : size(Fs,4)
            % calculations of fruppa -----------
            Kleftpart = Fs(:,:,i,j) * w;
            Kleftpart = Kleftpart * Fs(:,:,i,j)';
            Kleftpart = Kleftpart/norm(Kleftpart,'fro');
            
            % calculations of fruppa -----------
            Krightpart = epipole(Fs(:,:,i,j)) * w * epipole(Fs(:,:,i,j))';
            % for epipole reffer the function epipole -------
            Krightpart = Krightpart/norm(Krightpart, 'fro');
            
            % finding difference
            Kdifference = Kleftpart - Krightpart;
            
            % calculation of estimated error
            errestim = [errestim Kdifference(1,:) Kdifference(2,2:3)] ;
        
    end
end


end

%% ------------2.2 COST FUNC TO SIMPLIFIED KRUPPA's EQU----------------------

function errestim = costFuncKSimplified( Fs, p )
% Fs - given fundamental matrix ---------------
% initial intrinsic-values

A=[p(1) p(2) p(3); 0 p(4) p(5); 0 0 1];

w = A * A';

% ----
errestim = [];

for i = 1 : size(Fs,3) - 1
    for j = i+1 : size(Fs,3)
        
        % now finding svd of fundamental matrix
        [M,N,O] = svd(Fs(:,:,i,j));
        
        % read first first 3 colums M and O (after decomposed svd) 
        m1 = M(:,1);   m2 = M(:,2);   m3 = M(:,3);
        o1 = O(:,1);   o2 = O(:,2);   o3 = O(:,3);
        
        % taking the sigma values of N (decomposed svd) 
        rx = N(1,1);        ry = N(2,2);
        
        % building estimates for cost func -Cross multiplicartions Simp Krupps eq.---------
        x1 = -rx^2 * o1' * w ;
        x1 = x1 * (o1 * m1' * w * m2);
        x2 = rx * ry * o1' * w ;
        x2 = x2 * (o2 * m2' * w * m2);
        
        x3 = rx * ry * o1' * w ;
        x3 = x3 * (o2 * m1' * w * m1);
        x4 = -ry^2 * o2' * w ;
        x4 = x4 * (o2 * m1' * w * m2);
        
        x5 = rx^2 * o1' * w ;
        x5 = x5 * (o1 * m1' * w * m1);
        x6 = ry^2 * o2' * w ;
        x6 = x6 * (o2 * m2' * w * m2);
        
        % usig error equn...
        errestim = [errestim (x1-x2)/sqrt(x1^2 + x2^2) (x3-x4)/sqrt(x3^2 + x4^2) (x5-x6)/sqrt(x5^2 + x6^2)];
        
    end
end


end

