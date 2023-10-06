% Workspace initialisation
clear
close all
clc

% Loading DH table via 'init.m' script
init

% Jacobian matrix calculation
% NOTE : By default, the function analyses a configuration consisting only
% of rotoidal joints. If a different configuration is to be analysed, add 
% an array representing the configuration to the 'Jacobian' function as
% described in the body of the function.
J = Jacobian(DH);

% Matrix visualisation
disp(">>> Jacobian matrix:");
fprintf("\n");
disp(J);