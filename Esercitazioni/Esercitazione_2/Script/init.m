% To analyse different configurations, modify the file accordingly
% (joint variables, number of arms, etc.)

% Robotic arms length [m]
a1 = 0.3;
a2 = 0.5;
a3 = 0.3;

a = [a1 a2 a3]';

% Distance between O and O' on z-axis [m]
d1 = 0;
d2 = 0;
d3 = 0;

d =[d1 d2 d3]';

% Alfa angles [rad]
alfa1 = 0;
alfa2 = 0;
alfa3 = 0;

alfa = [alfa1 alfa2 alfa3]';

% Theta angles [rad]
theta1 = pi/3;
theta2 = pi/4;
theta3 = pi/4;

theta = [theta1 theta2 theta3]';

% Denavit-Hartenberg table

DH = [a d alfa theta];