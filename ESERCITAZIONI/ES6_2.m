% Inizializzo il workspace
clear
close all
clc

addpath("..\FUNCTIONS\");
addpath("..\DRAW_ROBOT\");
addpath(genpath("..\COPPELIA\"));

% Parametri temporali di simulazione
simTime = 7;
sampTime = 1e-2;
t = 0 : sampTime : simTime;
nPoints = length(t);

% Configurazione iniziale Jaco2
startConfig = [1.19 1.15 0.23 1.80 -1.29 2.00 3.69]';
DOF = length(startConfig);

% Matrici di salvataggio
q = zeros(DOF, nPoints);

% Parametri del profilo di velocità
endConfig = startConfig - 0.52*ones(7,1);
cruiseVel = 0.15;
finalTime = 5;

for i = 1 : nPoints
    
    [q(:,i), dq(:,i), ddq(:,i)] = trapezoidal( ...
        startConfig, ...
        endConfig, ...
        finalTime, ...
        cruiseVel, ...
        t(i));

end

conn = connect2coppelia(q, 7, 1e-2);


figure
subplot 311
plot(t, q, LineWidth=1.3);
xlabel("Tempo [s]");
ylabel("Posizione dei giunti [rad]");
grid on
subplot 312
plot(t, dq, LineWidth=1.3);
xlabel("Tempo [s]");
ylabel("Velocità dei giunti [rad/s]");
grid on
subplot 313
plot(t, ddq, LineWidth=1.3);
xlabel("Tempo [s]");
ylabel("Accelerazione dei giunti [rad/s^2]");
grid on















