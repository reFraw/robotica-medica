%% Main
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
DH = jaco2DH(startConfig);
DOF = length(startConfig);

% Matrici di salvataggio
q = zeros(DOF, nPoints);
dq = zeros(DOF, nPoints);
posErr = zeros(3, nPoints);
oriErr = zeros(3, nPoints);
positionEE = zeros(3, nPoints);

% Guadagni
pGain = 25;
oGain = 35;
K = diag([ ...
    pGain*[1 1 1], ...
    oGain*[1 1 1]]);

% Posizione e orientamento iniziali
T0 = DirectKinematics(DH);
pIni = T0(1:3,4,DOF);
quatIni = rot2quat(T0(1:3,1:3,DOF));

% Parametri per la pianificazione della traiettoria
cruiseVel = 0.07;
finalTime = 5;
pFin = pIni + [0 0 0.3]';
path = pFin - pIni;
normPath = norm(path);
desiredQuat = quatIni;

for i = 1 : nPoints
    
    % Posizione e orientamento correnti
    T = DirectKinematics(DH);

    currentPos = T(1:3,4,DOF);
    currentQuat = rot2quat(T(1:3,1:3,DOF));

    % Posizione desiderata
    [s, ds, dds] = trapezoidal( ...
        0, normPath, finalTime, cruiseVel, t(i));

    desiredPos = pIni + (s/normPath)*path;
    desiredLinVel = (ds/normPath)*path;
    desiredAngVel = zeros(3,1);
    feedForwardTerm = [desiredLinVel; desiredAngVel];

    % Calcolo dell'errore
    eP = desiredPos - currentPos;
    eO = quaternionError(desiredQuat, currentQuat);
    e = [eP; eO];

    posErr(:,i) = eP;
    oriErr(:,i) = eO;

    % Calcolo dello Jacobiano e pseudoinversa
    J = Jacobian(DH);
    invJ = pinv_damped(J);

    % IK
    dqi = invJ*(feedForwardTerm + K*e);
    dq(:,i) = dqi;
    q(:,i) = DH(:,4);

    qNext = q(:,i) + sampTime*dqi;

    if i < nPoints
        DH(:,4) = qNext;
    end

end

%% Connessione CoppeliaSim
conn = connect2coppelia(q, simTime, sampTime);

%% Plot degli errori
figure
subplot 211
plot(t, posErr, LineWidth=1.4);
xlabel("Tempo [s]");
ylabel("Errore [m]");
legend("X", "Y", "Z");
title("Errore di posizione");
grid on
subplot 212
plot(t, oriErr, LineWidth=1.4);
xlabel("Tempo [s]");
ylabel("Errore [u.a.]");
legend("\epsilon_x", "\epsilon_y", "\epsilon_z");
title("Errore di posizione");
grid on

%% Plot variabili di giunto
figure
subplot 211
plot(t, q, LineWidth=1.4);
xlabel("Tempo [s]");
ylabel("Angolo [rad]");
title("Posizione dei giunti");
grid on
subplot 212
plot(t, dq, LineWidth=1.4);
xlabel("Tempo [s]");
ylabel("Velocità [rad/s]");
title("Velocità dei giunti");
grid on
















