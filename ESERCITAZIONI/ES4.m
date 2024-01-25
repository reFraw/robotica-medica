%% Main script

% Inizializzo il workspace
clear
close all
clc

addpath("..\FUNCTIONS\");
addpath("..\DRAW_ROBOT\");
addpath("FILES\");

% Richiamo la configurazione iniziale del manipolatore
init_ES1;

% Definisco le variabili temporali di simulazione
Tf = 3;
Tc = 1e-3;
t = 0 : Tc : Tf;
nPoints = length(t);

% Definisco il numero di bracci 
nLink = length(DH(:,1));

% Definisco le matrici di salvataggio
q = zeros(nLink, nPoints);
dq = zeros(nLink, nPoints);
xe = zeros(nLink, nPoints);
errors = zeros(6, nPoints);
DHS = zeros(nLink, 4, nPoints);

% Definisco il set point desiderato
xd = [0.6 0.6 0]';
Rd = zyz2rot([pi/3 0 0]);
QD = rot2quat(Rd);

% Definisco la matrice dei guadagni
matrixK_P = diag([25 25 25]);
matrixK_O = diag([100 100 100]);

% Salvo la prima tabella DH
firstDH = DH;

inverseKinematicsType = "inverse";

for i = 1 : nPoints
    
    % Definisco il vettore corrente delle variabili di giunto
    q(:,i) = DH(:,4);
    
    % Calcolo la cinematica diretta
    T = DirectKinematics(DH);
    T0 = T(:,:,nLink);
    
    % Determino posizione e orientamento corrente dell'e.e
    % Aggiungo il valore corrente alla matrice delle pose
    xe_i = T0(1:3,4);
    rotM = T0(1:3,1:3);
    QE = rot2quat(rotM);
    xe(:,i) = xe_i;
    
    % Calcolo l'errore e lo salvo nella matrice degli errori
    eP = xd - xe_i;
    eO = quaternionError(QD, QE);
    errors(:,i) = [eP; eO];
    
    % Calcolo il Jacobiano geometrico
    J = Jacobian(DH);

    eTot = [matrixK_P*eP; matrixK_O*eO];

    if inverseKinematicsType == 'inverse'

        % Calcolo l'inversa/pseudoinversa dello Jacobiano
        invJ = pinv(J);
        
        % Calcolo q differenziale e lo salvo nella matrice
        dqi = invJ*eTot;
    else

        % Calcolo la trasposta dello Jacobiano
        J_trasp = transpose(J);
        
        % Calcolo q differenziale e lo salvo nella matrice
        dqi = J_trasp*eTot;
    end

    dq(:,i) = dqi;
    
    % Calcolo le nuove variabili di giunto
    qi = q(:,i) + Tc*dqi;
    
    % Salvo e aggiorno la tabella di Denavit-Hartenberg 
    DHS(:,:,i) = DH;
    DH(:,4) = qi;

end

finalJointConfiguration = rad2deg(q(:,end));

%% Plot degli errori
figure

subplot(2,1,1)
plot(t, errors(1,:), "LineWidth", 2, DisplayName="X error")
hold on
plot(t, errors(2,:), "LineWidth", 2, DisplayName="Y error")
plot(t, errors(3,:), "LineWidth", 2, DisplayName="Z error")
xlabel("Time [s]")
ylabel("Position errors [m]")
title("Position errors")
legend()
grid on

subplot(2,1,2)
plot(t, errors(3,:), "LineWidth", 2, DisplayName="\epsilon_x error")
hold on
plot(t, errors(4,:), "LineWidth", 2, DisplayName="\epsilon_y error")
plot(t, errors(5,:), "LineWidth", 2, DisplayName="\epsilon_z error")
xlabel("Time [s]")
ylabel("Orientation errors")
title("Orientation errors")
legend()
grid on

%% Plot della traiettoria e delle configurazioni
startPoint = xe(1:2,1);
endPoint = xd(1:2);

figure
DrawRobot(firstDH);
hold on
DrawRobot(DH)
p1 = plot(xe(1,:), xe(2,:), LineWidth=3, DisplayName="Traiettoria");
p2 = plot(startPoint(1), startPoint(2), Marker="o", MarkerSize=15, MarkerFaceColor="g", DisplayName="Start point");
p3 = plot(endPoint(1), endPoint(2), Marker="o", MarkerSize=15, MarkerFaceColor="r", DisplayName="Set point");
realplot = [p1 p2 p3];
legend(realplot, "Traiettoria", "Start point", "Set point")
grid on

%% Variabili di giunto definitive
clc
disp("Le variabali di giunto da considerare sono:");
fprintf("\n");
disp(finalJointConfiguration);