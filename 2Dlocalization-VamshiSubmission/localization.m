%% This LOCALIZATION.M is the main file of 2D GRID LOCALIZATION
% INSTRUCTION:: RUN "LOCALIZATION" in Command Window in order to Run the program
% INPUTS:  p = here uniform probabilities assigned for each cell
%          world = red and green cells spread environment
%          u = horizontal movement  ,  v = vertical movement
%          z = measurements vector for sensing
% OUTPUTS: qsense and qmove
%% PROGRAM STARTS HERE

clc
clear all
close all

% CREATING 2D-GRID ENVIRONMENT

% world = {  'R','G','G','G','G';
%            'G','G','G','G','G';
%            'G','G','G','G','G';
%            'G','G','G','G','G';
%            'G','G','G','G','G'  }
%UnComment below world :: Test using below world
world = {  'R','G','G','R','R';
    'R','G','G','R','G';
    'R','G','R','R','G';
    'R','G','R','G','R';
    'G','R','G','G','R'};
% Taking p here as probabiilties of uniform distribution in all grids
p = ones(5,5)/25;

% Here we can take multiple measurements
z={'R','R'};

%% MOVEMENT INPUT
% u and v are motion (shift) parameters
u = 2;
v = 0;
%% INSTRUCTIONS TO USER
fprintf('p-initial uniform probability distribution');
fprintf('\n');
disp(p)          %displaying uniform probabilities
q=p;

fprintf('\n');   %instruction about sense and move
fprintf('length of measurements='); disp(length(z))
fprintf('No.of moves = No. of sense');
fprintf('\n');

fprintf('\n');     % instruction to user
fprintf('Remember! one sense leads to one move and'); fprintf('\n');
fprintf('Also, one move leads to one sense');fprintf('\n');   fprintf('\n');

%% Plotting p :: since p is uniform :: plot not required
%colormap(gray);
%figure; imagesc(p);

%% SENSE AND MOVE ITERATIONS
% Looping for one sense and one move consecutively
for i = 1:length(z)
    % sense iteration
    q = twoDsense(q,z(i), world);
    fprintf('Sense considering Inaccuracy in iteration');
    disp(i);    fprintf('\n');  disp(q)     % priinting i th iteration sense
    figure;
    colormap(gray);  %plotting in gray scale
    imagesc(q);
    % move iteration
    q = twoDmove(q ,u , v);
    fprintf('Move considering Motion Error in iteration');
    disp(i);    fprintf('\n');    disp(q)   % priinting i th iteration move
    figure;  
    colormap(gray);   %plotting in gray scale
   imagesc(q);
end

%colormap(gray);
%figure;  imagesc(q);
%% PROGRAM END