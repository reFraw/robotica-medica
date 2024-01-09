%% MAIN

% Inizializzazione del workspace
clear
close all
clc

% Definizione path di interesse
addpath(genpath("..\COPPELIA\"));
addpath("..\DRAW_ROBOT\");
addpath("..\FUNCTIONS\");

% Upload configurazione iniziale Jaco2
startConfiguration = [1.1552 0.7108 0.1742 1.995 -1.5110 2.1265 4.3845]';
DH = jaco2DH(startConfiguration);

DOF = length(startConfiguration);

% Parametri temporali di simulazione
simTime = 7;
sampTime = 1e-2;
t = 0 : sampTime : simTime;
nPoints = length(t);

% Matrici di salvataggio
q = zeros(DOF, nPoints);
dq = zeros(DOF, nPoints);
orientationErrors = zeros(3, nPoints);

% Guadagni del manipolatore
pGain = 20;
oGain = 20;

K = diag([pGain*[1 1 1], oGain*[1 1 1]]);

% Posizione e orientamento iniziale
T_i = DirectKinematics(DH);

posIniziale = T_i(1:3,4,DOF);
oriIniziale = T_i(1:3,1:3,DOF);

rpyIniziale = rot2rpy(oriIniziale, 1);

% Parametri del profilo di velocità trapezoidale
cruiseVel = 1;
finalTime = 5;

% Parametri traiettoria
oriFinale = yRot(-pi/2);
rpyFinale = rot2rpy(oriFinale,1);

Rif = oriIniziale'*oriFinale;
[axis, finalTheta] = rot2axisangle(Rif);

path = rpyFinale - rpyIniziale;
normPath = norm(path);

% METHOD_FLAG 0 --> RPY angles
% METHOD_FLAG 1 --> Axis-angle

METHOD_FLAG = 1;

for i = 1 : nPoints

    currentTime = t(i);

    % Posizione e orientamento corrente
    T = DirectKinematics(DH);

    posCorrente = T(1:3,4,DOF);
    oriCorrente = T(1:3,1:3,DOF);
    quatCorrente = rot2quat(oriCorrente);

    if METHOD_FLAG == 0
        % Orientamento desiderato
        [s, ds, dds] = trapezoidal( ...
            0, ...
            normPath, ...
            finalTime, ...
            cruiseVel, ...
            currentTime);

        rpyDesiderato = rpyIniziale + (s/normPath)*path;
        rotDesiderato = rpy2rot(rpyDesiderato);
        quatDesiderato = rot2quat(rotDesiderato);

    else
        [theta, dtheta, ddtheta] = trapezoidal( ...
            0, ...
            finalTheta, ...
            finalTime, ...
            cruiseVel, ...
            currentTime);

        desiredRotMat = oriIniziale*axisangle2rot(axis, theta);
        quatDesiderato = rot2quat(desiredRotMat);

    end

    % Calcolo degli errori
    eO = quaternionError(quatDesiderato, quatCorrente);
    eP = zeros(3,1);
    e = [eP; eO];

    orientationErrors(:,i) = eO;

    % Calcolo dello Jacobiano
    J = Jacobian(DH);
    invJ = pinv_damped(J);

    % Algoritmo di inversione
    dqi = invJ*K*e;

    % Salvataggio dello stato attuale
    dq(:,i) = dqi;
    q(:,i) = DH(:,4);

    % Update dello stato successivo
    qNext = q(:,i) + sampTime*dqi;
    DH(:,4) = qNext;

end

%% Connessione CoppeliaSim
conn = connect2coppelia(q, simTime, sampTime);

%% Plot errore di orientamento
figure
plot(t, orientationErrors, LineWidth=1.5);
legend("\phi", "\theta", "\psi");
xlabel("Tempo [s]");
ylabel("Errore [u.a.]");
title("Errore di orientamento")
grid on

%% Plot variabili di giunto e velocità
figure

subplot 211
plot(t, q, LineWidth=1.2);
xlabel("Tempo [s]");
ylabel("Angolo [rad]");
title("Angolo di giunto")
legend("Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6", "Joint 7")
grid on

subplot 212
plot(t, dq, LineWidth=1.2)
xlabel("Tempo [s]");
ylabel("Velocità angolare [rad/s]");
title("Velocità angolari di giunto")
legend("Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6", "Joint 7")
grid on






