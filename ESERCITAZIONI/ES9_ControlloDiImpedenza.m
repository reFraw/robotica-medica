%% Main
% Inizializzazione del workspace
clear
close all
clc

% Variabili temporali di simulazione
simulationTime = 5;
samplingTime = 1e-2;
t = 0 : samplingTime : simulationTime;
nPoints = length(t);

% Matrici di salvataggio
position = zeros(nPoints,1);
velocity = zeros(nPoints,1);
acceleration = zeros(nPoints,1);

posErr = zeros(nPoints,1);
velErr = zeros(nPoints,1);

% Stato iniziale
position(1) = 0;
velocity(1) = 0;
acceleration(1) = 0;

% Guadagni
KP = 10;
KV = 10;
Md = 4;
K_EL = 100;

% Stima parametri dinamici e posizione desiderata
xd = 0.2;
xDotd = 0;
xDDotd = 0;

mHat = 3;
m = 3;

for i = 1 : nPoints

    currentTime = t(i);

    posErr(i) = xd - position(i);
    velErr(i) = xDotd - velocity(i);

    f = -K_EL*position(i);

    u = (mHat/Md)*(Md*xDDotd+KV*velErr(i)+KP*posErr(i)-f);

    if i < nPoints
        acceleration(i+1) = (u + f)/m;
        velocity(i+1) = velocity(i) + samplingTime*acceleration(i+1);
        position(i+1) = position(i) + samplingTime*velocity(i+1);
    end

end

%% Plot errori
figure
subplot 211
plot(t, posErr, LineWidth=1.4)
xlabel("Tempo [s]")
ylabel("Errore [m]")
title("Errore di posizione")
grid on
subplot 212
plot(t, velErr, LineWidth=1.4)
xlabel("Tempo [s]")
ylabel("Errore [m/s]")
title("Errore di velocità")
grid on