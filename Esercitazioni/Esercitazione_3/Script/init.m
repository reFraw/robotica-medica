%% INITIALIZATION SCRIPT

% Length of robot arms [m]
a1 = 1;
a2 = 1;
a3 = 1;

a = [a1 a2 a3]';

% Alpha angles [rad]
alpha1 = 0;
alpha2 = 0;
alpha3 = 0;

alpha = [alpha1 alpha2 alpha3]';

% Distance between O and O' [m]
d1 = 0;
d2 = 0;
d3 = 0;

d = [d1 d2 d3]';

% Theta angles [rad];
theta1 = pi/2;
theta2 = -pi/2;
theta3 = -pi/2;

theta = [theta1 theta2 theta3]';

% Denavit-Hartenberg's table
DH = [a alpha d theta];