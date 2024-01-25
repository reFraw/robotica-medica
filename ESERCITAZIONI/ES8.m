% Inizializzazione del workspace
clear
close all
clc

% Aggiunta dei percorsi di interesse
addpath("..\JACO2_DH\");
addpath("..\FUNCTIONS\");
addpath("..\DRAW_ROBOT\");
addpath(genpath("..\COPPELIA"));

% Upload configurazione iniziale Jaco2
startConfiguration = [2.2738 0.1837 -0.3007 1.3661 0.0583 1.1910 5.7329]';
DH = jaco2DH(startConfiguration);

DOF = length(startConfiguration);

% Parametri temporali di simulazione
simTime = 7;
samplingTime = 1e-2;
t = 0 : samplingTime : simTime;
nPoints = length(t);

% Gaudagno del manipolatore
posGain = 30;
oriGain = 30;

gainMatrix = diag([posGain*[1 1 1], oriGain*[1 1 1]]);

% Determinazione posizione e orientamento iniziale E.E.
T = DirectKinematics(DH);

startPositionEE = T(1:3, 4, DOF);
startRotationMatrix = T(1:3, 1:3, DOF);

% Matrici di salvataggio dati
q = zeros(DOF, nPoints);
dq = zeros(DOF, nPoints);
positionErrors = zeros(3, nPoints);
orientErrors = zeros(3, nPoints);
positionEE = zeros(3, nPoints);
DHS = zeros(DOF, 4, 2);

q(:,1) = startConfiguration;
PositionEE(:,1) = startPositionEE;
DHS(:,:,1) = DH;

% Parametri traiettoria circolare
radius = 0.1;
center = startPositionEE + [radius 0 0]';
rCircle = eye(3);
startPoint = radius*pi;
endPoint = 2*radius*pi;

cruiseVelPos = 0.1;
finalTimePos = 5;

% Parametri traiettoria orientamento
firstRotation = zRot(pi);
secondRotation = yRot(pi);
fullRotation = secondRotation*firstRotation;
Rif = startRotationMatrix'*fullRotation;

[axis, finalTheta] = rot2axisangle(Rif);

cruiseVelOr = 1;
finalTimeOr = 5;

for i = 1 : nPoints

    currentTime = t(i);

    % Determino posizione e orientamento corrente
    T0 = DirectKinematics(DH);
    currentPosition = T0(1:3,4,DOF);
    currentOrientMatrix = T0(1:3,1:3,DOF);
    currentQuat = rot2quat(currentOrientMatrix);

    positionEE(:,i) = currentPosition;

    % Pianificazione traiettoria per la posizione dell'E.E.
    [s, ds, dds] = trapezoidal( ...
        startPoint, ...
        endPoint, ...
        finalTimePos, ...
        cruiseVelPos, ...
        currentTime);

    desiredPos = center + rCircle*[radius*cos(s/radius) radius*sin(s/radius) 0]';
    desiredVel = rCircle*[-ds*sin(s/radius) ds*cos(s/radius) 0]';

    % Pianificazione traiettoria per l'orientamento dell'E.E.
    [theta, dotheta, ddotheta] = trapezoidal( ...
        0, ...
        finalTheta, ...
        finalTimeOr, ...
        cruiseVelOr, ...
        currentTime);

    desiredRotMat = startRotationMatrix*axisangle2rot(axis, theta);
    desiredQuat = rot2quat(desiredRotMat);

    % Calcolo degli errori
    eP = desiredPos - currentPosition;
    eO = quaternionError(desiredQuat, currentQuat);
    e = [eP; eO];

    positionErrors(:,i) = eP;
    orientErrors(:,i) = eO;

    % Calcolo e pseudo inversa dello Jacobiano
    J = Jacobian(DH);
    invJ = pinv_damped(J);

    % Calcolo del dq
    dqi = invJ*gainMatrix*e;

    dq(:,i) = dqi;

    % Calcolo delle variabili di giunto per t+dt
    qNext = q(:,i) + samplingTime*dqi;
    
    if i < nPoints

        q(:,i+1) = qNext;
        DH(:,4) = qNext;

    end

end

DHS(:,:,2) = DH;

%% Connessione CoppeliaSim
conn = connect2coppelia(q, simTime, samplingTime);

%% Plot
% Aggiungere eventuali plot di interesse

figure();
plot(t, positionErrors, LineWidth=1.5);
grid on
xlabel("Tempo [s]");
ylabel("Errore [m]");
title("Errore di posizione");
legend("Coordinata X", "Coordinata Y", "Coordinata Z");
set(gca, "FontSize", 12);

figure();
plot(t, orientErrors, LineWidth=1.5);
grid on
xlabel("Tempo [s]");
ylabel("Errore [u.a.]");
title("Errore di orientamento");
legend("\epsilon_x", "\epsilon_y", "\epsilon_z");
set(gca, "FontSize", 12);

%% Configurazione iniziale e finale
figure
DrawRobot(DHS(:,:,1))
plot3(positionEE(1,:), positionEE(2,:), positionEE(3,:), LineWidth=2);
DrawRobot(DHS(:,:,2))
grid on













