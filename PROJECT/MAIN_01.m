%% Main 
% Inizializzazione del workspace
clear
close all
clc

addpath("..\FUNCTIONS\");
addpath("..\DRAW_ROBOT\");
addpath(genpath("..\COPPELIA\"));

% Variabili temporali di simulazione
simulationTime = 5;
samplingTime = 1e-2;
t = 0 : samplingTime : simulationTime;
nPoints = length(t);

% Configurazione iniziale
startConfiguration = [2.274 0.184 -0.300 1.366 0.058 1.191 5.733]';
DH = jaco2DH(startConfiguration);
DOF = length(startConfiguration);

% Posizione e orientamento iniziali
T0 = DirectKinematics(DH);
pIni = T0(1:3,4,DOF);
Ri = T0(1:3,1:3,DOF);

% Posizione e orientamento finali
pFin = [0.5 -0.2 0.6]';
path = pFin - pIni;
normPath = norm(path);

Rf = xRot(pi/2);
Rif = Ri'*Rf;
[axis, thetaFin] = rot2axisangle(Rif);

% Guadagni del manipolatore
pGain = 35;
oGain = 55;
K = diag([ ...
    pGain*[1 1 1], ...
    oGain*[1 1 1]]);

% Variabili di salvataggio
q = zeros(DOF, nPoints);
dq = zeros(DOF, nPoints);
posErr = zeros(3, nPoints);
oriErr = zeros(3, nPoints);
positionEE = zeros(3, nPoints);
DHS = zeros(DOF, 4, 2);

DHS(:,:,1) = DH;

for i = 1 : nPoints
    currentTime = t(i);
    
    % Posizione e orientamento corrente
    T = DirectKinematics(DH);
    currentPos = T(1:3,4,DOF);
    currentQuat = rot2quat(T(1:3,1:3,DOF));

    positionEE(:,i) = currentPos;

    % Pianificazione della posizione
    [s, ds, dds] = trapezoidal( ...
        0, normPath, 4, 1, currentTime);

    desiredPos = pIni + (s/normPath)*path;

    % Pianificazione dell'orientamento
    [th, dth, ddth] = trapezoidal( ...
        0, thetaFin, 4, 1, currentTime);

    Rt = Ri*axisangle2rot(axis, th);
    desiredQuat = rot2quat(Rt);

    % Calcolo degli errori
    eP = desiredPos - currentPos;
    eO = quaternionError(desiredQuat, currentQuat);
    e = [eP;eO];

    posErr(:,i) = eP;
    oriErr(:,i) = eO;

    % Inversione cinematica
    J = Jacobian(DH);
    invJ = pinv_damped(J);

    dqi = invJ*K*e;
    dq(:,i) = dqi;
    q(:,i) = DH(:,4);

    qNext = q(:,i) + samplingTime*dqi;

    if i < nPoints
        DH(:,4) = qNext;

    end
   
end

DHS(:,:,2) = DH;
finalJointConfiguration = DH(:,4);
save("OUTPUT_FILE\jointConfigurtion_MAIN01.mat", "finalJointConfiguration");

%% Connessione CoppeliaSim
conn = connect2coppelia(q, simulationTime, samplingTime);

%% Plot delle variabili
figure
subplot 211
plot(t, posErr, LineWidth=1.3);
xlabel("Tempo [s]");
ylabel("Errore [m]");
title("Errore di posizione");
legend("X", "Y", "Z");
grid on
subplot 212
plot(t, oriErr, LineWidth=1.3);
xlabel("Tempo [s]");
ylabel("Errore [u.a.]");
title("Errore di orientamento");
legend("\phi", "\theta", "\psi");
grid on

figure
subplot 211
plot(t, q, LineWidth=1.3);
xlabel("Tempo [s]");
ylabel("Angolo [rad]");
title("Variabili di giunto");
grid on
subplot 212
plot(t, dq, LineWidth=1.3);
xlabel("Tempo [s]");
ylabel("Velocità [rad/s]");
title("Velocità di giunto");
grid on

figure
DrawRobot(DHS(:,:,1))
hold on
DrawRobot(DHS(:,:,2))
plot3(positionEE(1,:), positionEE(2,:), positionEE(3,:), LineWidth=1.3);
grid on


