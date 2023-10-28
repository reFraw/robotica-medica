%% Input parameters

clear 
close all 
clc

%%%%% INPUT PARAMETERS %%%%%
pD = [0.5 0.3 0.4]';                     % [X Y Z]
oD = [pi/6 pi/4 pi/4]';                  % [phi theta psi] - ZYZ
start = [pi/6 0 pi/2 pi/2 pi/4 0 0]';    % Initial set-up (Joint variables)
simulationTime = 5;                      % Simulation time [s]
positionGain = 2;                        % Position gain for KP matrix
orientationGain = 2;                     % Orientation gain for KO matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[q, dq, pErr, oErr, path, DHS] = inverseKinematics( ...
    pD, ...
    oD, ...
    start, ...
    simulationTime, ...
    positionGain, ...
    orientationGain);

%% Errors plot

figure

subplot 211
plot(pErr(4,:), pErr(1,:), LineWidth=2);
hold on
plot(pErr(4,:), pErr(2,:), LineWidth=2);
plot(pErr(4,:), pErr(3,:), LineWidth=2);
legend("X error", "Y error", "Z error")
grid on

subplot 212
plot(oErr(4,:), oErr(1,:), LineWidth=2);
hold on
plot(oErr(4,:), oErr(2,:), LineWidth=2);
plot(oErr(4,:), oErr(3,:), LineWidth=2);
legend("\phi error", "\theta error", "\psi error")
grid on

%% Path visualization

tolerance = 1e-3;
endPoint = path(:,end);
startPoint = path(:,1);

figure
DrawRobot(DHS(:,:,1));
hold on
DrawRobot(DHS(:,:,end))
if norm(pD - endPoint) <= tolerance
    p1 = plot3(path(1,:), path(2,:), path(3,:), LineWidth=2, Color="g");
else
    p1 = plot3(path(1,:), path(2,:), path(3,:), LineWidth=2, Color="r");
end
p2 = plot3(path(1,1), path(2,1), path(3,1), "o", MarkerSize=20, MarkerFaceColor="b");
p3 = plot3(endPoint(1), endPoint(2), endPoint(3), "o", MarkerSize=20, MarkerFaceColor="r");
p4 = plot3(pD(1), pD(2), pD(3), "o", MarkerSize=20, MarkerFaceColor="g");
grid on
legend([p1 p2 p3 p4], "Path", "Start point", "End point", "Set point")