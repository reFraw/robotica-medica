clear
close all
clc

addpath("..\FUNCTIONS\");
addpath("..\DRAW_ROBOT\");
addpath("FILES\");

init_ES1;

T = DirectKinematics(DH);
disp(T(:,:,end));

DrawRobot(DH);