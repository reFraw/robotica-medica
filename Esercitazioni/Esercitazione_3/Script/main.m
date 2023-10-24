%% Main script

% Inizializzo il workspace
clear
close all
clc

% Richiamo la configurazione iniziale del manipolatore
init

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
errors = zeros(nLink, nPoints);
DHS = zeros(nLink, 4, nPoints);

% Definisco il set point desiderato
xd = [1.8 0.6 pi/8]';

% Definisco la matrice dei guadagni
matrixK = diag([25 25 25]);

% Salvo la prima tabella DH
firstDH = DH;

inverseKinematicsType = "inverse";

for i = 1 : nPoints
    
    % Definisco il vettore corrente delle variabili di giunto
    q(:,i) = DH(:,4);
    
    % Calcolo la cinematica diretta
    T = DirectKinematics(DH);
    T0 = T(:,:,nLink);
    
    % Determino posizione e orientamento corrente dell'e.e come [x, y, phi]'
    % Aggiungo il valore corrente alla matrice delle pose
    xe_i = [T(1:2,4,nLink); sum(DH(:,4))];
    xe(:,i) = xe_i;
    
    % Calcolo l'errore e lo salvo nella matrice degli errori
    ei = xd - xe_i;
    errors(:,i) = ei;
    
    % Calcolo il Jacobiano geometrico ed estraggo le righe funzionali
    J = Jacobian(DH);
    J = J([1:2 6], :);

    if inverseKinematicsType == 'inverse'
    
        % Calcolo l'inversa dello Jacobiano
        invJ = inv(J);
        
        % Calcolo q differenziale e lo salvo nella matrice
        dqi = invJ*matrixK*ei;
        dq(:,i) = dqi;

    else

        % Calcolo la trasposta dello Jacobiano
        J_trasp = transpose(J);
        
        % Calcolo q differenziale e lo salvo nella matrice
        dqi = J_trasp*matrixK*ei;
        dq(:,i) = dqi;
    end
    
    % Calcolo le nuove variabili di giunto
    qi = q(:,i) + Tc*dqi;
    
    % Salvo e aggiorno la tabella di Denavit-Hartenberg 
    DHS(:,:,i) = DH;
    DH(:,4) = qi;

end

finalJointConfiguration = rad2deg(q(:,end));

%% Plot degli errori
figure
title("Error")

subplot(1,2,1)
plot(t, errors(1,:), "LineWidth", 2, DisplayName="X error")
hold on
plot(t, errors(2,:), "LineWidth", 2, DisplayName="Y error")
legend()
grid on

subplot(1,2,2)
plot(t, errors(3,:), "LineWidth", 2, DisplayName="Theta error")
legend()
grid on

%% Plot della traiettoria e delle configurazioni
startPoint = xe(1:2,1);
endPoint = xd(1:2);

figure
DrawRobot(firstDH, 1);
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












