clear
close all
clc

addpath("..\FUNCTIONS\");

% La traccia richiede il calcolo del cinematica diretta per un manipolatore
% planare a 2 bracci.
% L'implementazione considera lo Jaco2, alternativamente implementare la
% matrice DH custom in un file init.m

jointVar = [-pi/4 0 0 -pi/3 pi/3 pi/4 pi/2]';
DH = jaco2DH(jointVar);

J = Jacobian(DH);

fprintf(">>> Jacobian matrix:\n\n");
disp(J);