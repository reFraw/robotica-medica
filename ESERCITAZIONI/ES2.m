clear
close all
clc

addpath("..\FUNCTIONS\");
addpath("..\DRAW_ROBOT\");
addpath("FILES\");

init_ES1;

J = Jacobian(DH);
disp(J);