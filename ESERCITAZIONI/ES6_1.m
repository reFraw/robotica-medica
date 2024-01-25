% Inizializzo il workspace
clear
close all
clc

addpath("..\FUNCTIONS\");

t = 0 : 1e-3 : 5;

pi = [-2 6 3];
pf = [-4 1 7];

finalTime = 3;
cruiseVelocity = [0.88 -2 2.666];

for i = 1 : length(t)
    
    [p(:,i), v(:,i), a(:,i)] = trapezoidal( ...
        pi, ...
        pf, ...
        finalTime, ...
        cruiseVelocity, ...
        t(i));

end

figure
subplot 311
plot(t, p, LineWidth=1.3);
grid on
xlabel("Tempo [s]");
ylabel("S [m]");
legend("Profilo 2\rightarrow4", "Profilo 6\rightarrow1", "Profilo 3\rightarrow7")
subplot 312
plot(t, v, LineWidth=1.3);
grid on
xlabel("Tempo [s]");
ylabel("V [m/s]");
legend("Profilo 2\rightarrow4", "Profilo 6\rightarrow1", "Profilo 3\rightarrow7")
subplot 313
plot(t, a, LineWidth=1.3);
grid on
xlabel("Tempo [s]");
ylabel("A [m/s^2]");
legend("Profilo 2\rightarrow4", "Profilo 6\rightarrow1", "Profilo 3\rightarrow7")