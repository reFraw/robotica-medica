clear
close all
clc

addpath("functions")
addpath("config")

qIn = [1 3 4];
qOut = [4 5 10];
finalTime = 1;
cruiseVel = 1.5;

t = 0 : 1e-4 : 5;

trajectory = zeros(length(qIn)*3, length(t));

for i = 1 : length(t)

    [p, v, a] = trapezoidal( ...
        qIn, ...
        qOut, ...
        finalTime, ...
        cruiseVel, ...
        t(i));

    vec = [p; v; a];

    trajectory(:,i) = vec;

end

%% 
figure

subplot 311
plot(t, trajectory(1,:), LineWidth=2);
title("Position")
grid on

subplot 312
plot(t, trajectory(4,:), LineWidth=2);
title("Velocity")
grid on

subplot 313
plot(t, trajectory(7,:), LineWidth=2);
title("Acceleration")
grid on