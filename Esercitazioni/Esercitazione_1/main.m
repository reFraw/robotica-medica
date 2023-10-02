% Workspace initialisation
clear
close all
clc

% Loading the Denavit-Hartenberg table by calling the 'init' script
init

% Calculation of homogeneous transformation matrices between the i-th
% triplet and the base triplet.
T = DirectKinematics(DH);

% Configuration display
DrawRobot(DH)