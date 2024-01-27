%% Main
% Inizializzazione del workspace
clear
close all
clc

addpath("..\FUNCTIONS\");
addpath("..\DRAW_ROBOT\");
addpath("AUX_SCRIPT\");
addpath(genpath("..\COPPELIA\"));

% Variabili temporali di simulazione
simulationTime = 10;
phaseOneTime = simulationTime / 2;
delay = 0.5;
samplingTime = 1e-2;
t = 0 : samplingTime : simulationTime;
nPoints = length(t);

% Configurazione iniziale
startConfiguration = load("OUTPUT_FILE\jointConfigurtion_MAIN01.mat");
startConfiguration = startConfiguration.finalJointConfiguration;
DH = jaco2DH(startConfiguration);
DOF = length(startConfiguration);

% Posizione e orientamento iniziali
T0 = DirectKinematics(DH);
pIni = T0(1:3,4,DOF);
Ri = T0(1:3,1:3,DOF);
QuatIni = rot2quat(Ri);

% Posizione e orientamento finali - Fase 1
pTarget = [0.4 -0.3 0.2]';
delta = [0 0 0.1]';
pFin = pTarget + delta;
path = pFin - pIni;
normP = norm(path);

Rf = xRot(pi);
Rif = Ri'*Rf;
[axis, thetaFin] = rot2axisangle(Rif);

% Guadagni del manipolatore
pGain = 55;
oGain = 55;
K = diag([ ...
    pGain*[1 1 1], ...
    oGain*[1 1 1]]);
k0 = 150;

% Variabili di salvataggio
q = zeros(DOF, nPoints);
dq = zeros(DOF, nPoints);
posErr = zeros(3, nPoints);
oriErr = zeros(3, nPoints);
positionEE = zeros(3, nPoints);
DHS = zeros(DOF, 4, 2);
pTargetMove1 = pTarget(3).*ones(length(0 : samplingTime : phaseOneTime-samplingTime),1);
manipulability = zeros(nPoints,1);
minSVD = zeros(nPoints,1);
maxSVD = zeros(nPoints,1);

for i = 1 : nPoints
    currentTime = t(i);
    
    % Posizione e orientamento correnti
    T = DirectKinematics(DH);
    currentPos = T(1:3,4,DOF);
    currentQuat = rot2quat(T(1:3,1:3,DOF));

    positionEE(:,i) = currentPos;

    if currentTime < phaseOneTime
        % Pianificazione posizione - Fase 1
        [s, ds, dds] = trapezoidal( ...
            0, normP, phaseOneTime-delay, 1, currentTime);

        desiredPos = pIni + (s/normP)*path;
        desiredVelocity = zeros(3,1);

        % Pianificazione orientamento - Fase 1
        [th , dth, ddth] = trapezoidal( ...
            0, thetaFin, phaseOneTime-delay, 1, currentTime);

        Rt = Ri*axisangle2rot(axis, th);
        desiredQuat = rot2quat(Rt);
        desiredAngularVelocity = zeros(3,1);

        lastIndex = i;
        
    else
        % Traslazione temporale per evitare sfasamento tra movimento
        % dell'E.E. e movimento del punto target.
        traslatedTime = currentTime - phaseOneTime;

        % Pianificazione posizione - Fase 2
        desiredPos = pTarget + delta + [0 0 0.05*sin(2*pi*traslatedTime)]';
        desiredVel = [0 0 2*pi*0.05*cos(2*pi*traslatedTime)]';
        pTargetMove2(i-lastIndex) = desiredPos(3) - 0.1;

        % Pianificazione orientamento - Fase 2
        desiredAngularVel = zeros(3,1);

    end

    % Calcolo errori
    eP = desiredPos - currentPos;
    eO = quaternionError(desiredQuat, currentQuat);
    e = [eP;eO];

    posErr(:,i) = eP;
    oriErr(:,i) = eO;

    % Inversione cinematica
    feedForwardTerm = [desiredVelocity; desiredAngularVelocity];

    J = Jacobian(DH);
    invJ = pinv_damped(J);

    % Calcolo del minimo e massimo valor singolare di J
    S = svd(J);
    minSVDvalue = min(S);
    minSVD(i) = minSVDvalue;
    maxSVDvalue = max(S);
    maxSVD(i) = maxSVDvalue;
    
    % Calcolo della manipolabilità
    manipulability(i) = computeManipulability(DH(:,4));

    % Calcolo del gradiente di w
    dWdQ = computeJointGradient(DH(:,4), 1);

    % Definizione della velocità da proiettare nel kernel di J
    qDot0 = k0*dWdQ;

    kernelProjector = eye(7) - invJ*J;
    dqi = invJ*(feedForwardTerm + K*e) + kernelProjector*qDot0;
    dq(:,i) = dqi;
    q(:,i) = DH(:,4);

    qNext = q(:,i) + samplingTime*dqi;

    if i < nPoints
        DH(:,4) = qNext;
    end

end

pTargetMove2 = pTargetMove2';
pTargetMove = [pTargetMove1; pTargetMove2];

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
legend("\epsilon_x", "\epsilon_y", "\epsilon_z");
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
subplot 211
plot(t, positionEE(3,:), LineWidth=1.3, DisplayName="Movimento E.E.");
hold on
plot(t, pTargetMove, LineWidth=1.3, DisplayName="Movimento p_{target}");
xlabel("Tempo [s]")
ylabel("Posizione [m]");
title("Movimento lungo asse Z");
legend()
grid on
subplot 212
plot(t, positionEE(3,:)'-pTargetMove, LineWidth=1.3);
xlabel("Tempo [s]");
ylabel("Scostamento \Delta s [m]");
ylim([0 max(positionEE(3,:)'-pTargetMove)+0.1])
title("Scostamento E.E. - p_{target}");
grid on

figure
plot(t, manipulability/max(manipulability), LineWidth=1.3);
xlabel("Tempo [s]")
ylabel("Manipolabilità");
title("Andamento della manipolabilità normalizzata - k_0=150")
grid on

figure
plot(t, minSVD, LineWidth=1.4, DisplayName="Minimo valor singolare");
hold on
plot(t, maxSVD, LineWidth=1.4, DisplayName="Massimo valor singolare");
xlabel("Tempo [s]");
title("Andamento del minimo e massimo valor singolare di J");
legend()
grid on

checkManipulability;
%% Funzioni ausiliarie

function w = computeManipulability(jointConfig)
    
    DH = jaco2DH(jointConfig);
    J = Jacobian(DH);
    S = svd(J);
    w = S(end)/S(1);

end

function dWdQ = computeJointGradient(jointConfig, h)

    nComponents = length(jointConfig);

    dWdQ = zeros(nComponents,1);

    for i = 1 : nComponents

        hi = zeros(7,1);
        hi(i) = h;

        wPlus = computeManipulability(jointConfig+hi);
        wMinus = computeManipulability(jointConfig-hi);

        dWdQ(i) = (wPlus-wMinus)/(2*h);

    end

end