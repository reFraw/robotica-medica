m0 = load("OUTPUT_FILE\m0.mat");
m0 = m0.manipulability;
m0 = m0 / max(m0);

mMax = load("OUTPUT_FILE\mMax.mat");
mMax = mMax.manipulability;
mMax = mMax / max(mMax);

figure
subplot 211
plot(t, m0, DisplayName="k_0=0");
hold on
plot(t, mMax, DisplayName="k_0=150");
grid on
legend()
xlabel("Tempo [s]")
ylabel("Manipolabilità normalizzata");
subplot 212
plot(t,mMax-m0, LineWidth=1.3);
grid on
xlabel("Tempo [s]")
ylabel("\DeltaM");


