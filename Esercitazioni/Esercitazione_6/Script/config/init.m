% Denavit-Hartenberg table

a = zeros(7,1);
alfa = zeros(7,1);
alfa(1:6) = pi/2;
d = [0.2755 0 -0.41 -0.0098 -0.3072 0 0.25]';

DH = [a alfa d];