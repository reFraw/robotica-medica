% This 'init' file is configured for a 3-arm spherical planar manipulator.
% To analyse different configurations, modify the parameters appropriately,
% remembering that given N the number of arms then the Denavit-Hartenberg
% table will have N rows and 4 columns.

% Length of robotic arms [m]

a1 = 0.4;
a2 = 0.3;
a3 = 0.5;

% Alpha angles [rad]

alfa1 = 0;
alfa2 = 0;
alfa3 = 0;

% Distance along z between the origin of the triad O and that O'
% on the axis of the previous joint [m]

d1 = 0;
d2 = 0;
d3 = 0;

% Theta angles [rad]

theta1 = pi/4;
theta2 = -pi/6;
theta3 = pi/3;

% Denavit-Hartenberg table

DH = [
    a1, alfa1, d1, theta1;
    a2, alfa2, d2, theta2;
    a3, alfa3, d3, theta3
    ];
