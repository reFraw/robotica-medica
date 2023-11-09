%% Blocco di pianificazione

% Inizializzazione
clear
close all
clc

% Definizione dei percorsi delle funzioni
addpath("config\");
addpath("functions\");
addpath("functions_coppelia\");
addpath("vrepScene\")

% Definizione della configurazione iniziale
init;

startConfiguration = [1.19 1.15 0.23 1.80 -1.29 2.00 3.69]';
DH = [DH startConfiguration];

nJoints = length(startConfiguration);

% Variabili temporali di simulazione [s]
simulationTime = 10;
samplingTime = 1e-2;
t = 0 : samplingTime : simulationTime;
nPoints = length(t);

% Matrici di guadagno
positionGain = 200;
orientationGain = 200;
KP = diag([positionGain positionGain positionGain]);
KO = diag([orientationGain orientationGain orientationGain]);

% Matrici di salvataggio
positionEE = zeros(3, nPoints);
orientationEE = zeros(3, nPoints);
q = zeros(nJoints, nPoints);
dq = zeros(nJoints, nPoints);
posError = zeros(3, nPoints);
orientError = zeros(3, nPoints);
realMotion = zeros(3, nPoints);
plannedMotion = zeros(3, nPoints);
plannedVelocity = zeros(3, nPoints);
plannedAcceleration = zeros(3, nPoints);
DHS = zeros(nJoints, 4, 2);
DHS(:,:,1) = DH;

% Definizione della traiettoria
T = DirectKinematics(DH);

startPoint = T(1:3, 4, nJoints);
endPoint = startPoint + [0 0 0.3]';
segmentNorm = norm(endPoint - startPoint);

cruiseVel = 0.07;
finalTime = 5;

desiredOrientEE = T(1:3, 1:3, nJoints);
desiredOrientEE = rot2quat(desiredOrientEE);

% Main control loop
for i = 1 : nPoints
    
    % Istante temporale corrente
    currentTime = t(i);
    
    % Calcolo della matrice di trasformazione omogenena tra base ed E.E.
    T0E = DirectKinematics(DH);
    
    % Calcolo della posizione corrente dell'E.E.
    currentPosEE = T0E(1:3, 4, nJoints);
    
    % Calcolo dell'orientamento corrente dell'E.E.
    currentOrientEE = T0E(1:3, 1:3, nJoints);
    currentOrientEE = rot2quat(currentOrientEE);
    
    % Calcolo della posizione e velocità desiderata all'istante corrente
    [S, dotS, ddotS] = trapezoidal( ...
        0, ...
        segmentNorm, ...
        finalTime, ...
        cruiseVel, ...
        currentTime);

    desiredPosEE = startPoint + (S/segmentNorm)*(endPoint - startPoint);
    
    desiredAngularVelEE = 0;
    
    % Calcolo dell'errore di posizione e orientamento
    ePi = desiredPosEE - currentPosEE;
    eOi = quatError(currentOrientEE, desiredOrientEE);
    
    %Salvataggio della posizione corrente e della posizione desiderata
    %corrente
    realMotion(:,i) = currentPosEE;

    plannedMotion(:,i) = desiredPosEE;
    plannedVelocity(:,i) = dotS;
    plannedAcceleration(:,i) = ddotS;
    
    % Salvataggio degli errori di posizione e orientamento
    posError(:,i) = ePi;
    orientError(:,i) = eOi;

    % Salvataggio delle variabili di giunto correnti
    q(:,i) = DH(:,4);
    
    % Calcolo dell'errore amplificato
    amplifiedErr = [dotS + KP*ePi; desiredAngularVelEE + KO*eOi];
    
    % Calcolo dello Jacobiano geometrico e inversione
    J = Jacobian(DH);
    pinvJ = pinv_damped(J);
    
    % Calcolo del dq corrente e salvataggio
    dqi = pinvJ*amplifiedErr;
    dq(:,i) = dqi;
    
    % Calcolo delle variabili di giunto per l'istante successivo 
    qNext = q(:,i) + samplingTime*dqi;
    
    % Salvataggio delle variabili di giunto e della configurazione attuale
    DH(:,4) = qNext;
    DHS(:,:,2) = DH;

end

%% Plot degli errori
f1 = figure;

subplot 211
plot(t, posError(1,:), LineWidth=2, DisplayName="X Error");
hold on
plot(t, posError(2,:), LineWidth=2, DisplayName="Y Error");
plot(t, posError(3,:), LineWidth=2, DisplayName="Z Error");
grid on
title("Position errors");
ylabel("Error [m]");
legend();

subplot 212
plot(t, orientError(1,:), LineWidth=2, DisplayName="\phi Error");
hold on
plot(t, orientError(2,:), LineWidth=2, DisplayName="\theta Error");
plot(t, orientError(3,:), LineWidth=2, DisplayName="\psi Error");
grid on
title("Orientation errors");
ylabel("Error [rad]");
legend();

h1 = axes(f1, 'visible', 'off');
h1.XLabel.Visible='on';
xlabel(h1,'Time [s]');

%% Plot delle velocità di giunto
f2 = figure;

for i = 1 : nJoints
    subplot(nJoints, 1, i);
    plot(t, dq(i,:), LineWidth=1.5, DisplayName="Joint "+num2str(i));
    legend()
    grid on
end

h2 = axes(f2, 'visible', 'off'); 
h2.Title.Visible='on';
h2.XLabel.Visible='on';
h2.YLabel.Visible='on';
ylabel(h2,'Angular velocity \omega [rad/s]');
xlabel(h2,'Time [s]');
title(h2,'Joint velocities');

%% Plot delle configurazioni iniziali e finali
figure

DrawRobot(DHS(:,:,1));
hold on
DrawRobot(DHS(:,:,2));
p1 = plot3(startPoint(1), startPoint(2), startPoint(3), "o", MarkerSize=20, ...
    MarkerFaceColor="g");
p2 = plot3(endPoint(1), endPoint(2), endPoint(3), "o", MarkerSize=20, ...
    MarkerFaceColor="r");
grid on
p3 = plot3(plannedMotion(1,:), plannedMotion(2,:), plannedMotion(3,:), ...
    LineWidth=2);
p4 = plot3(realMotion(1,:), realMotion(2,:), realMotion(3,:), ...
    LineWidth=2);
legend([p1 p2 p3 p4], "Start point", ...
    "End point", ...
    "Traiettoria pianificata", ...
    "Traiettoria reale");
grid on

%% Blocco di simulazione - CoppeliaSim

% Variabili di simulazione CoppeliaSim
coppeliaSimTime = 15;

connect = connect2coppelia(q, coppeliaSimTime);