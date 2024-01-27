T = DirectKinematics(DH);
c = T(1:3,4,end);
cx = c(1);
cy = c(2);
cz = c(3);

J = Jacobian(DH);
jPos = J(1:3,:);
MPOS = inv(jPos*jPos');

[eigenvec, eigenval] = eig(MPOS);
[axiss, angle] = rot2axisangle(eigenvec);


% Calcola l'ellissoide di manipolabilità considerando solo le velocità lineari
ellipsoid_points_linear = svd(MPOS);

% Plotta l'ellissoide di manipolabilità in uno spazio tridimensionale
figure;
[X,Y,Z] = ellipsoid(cx,cy,cz,ellipsoid_points_linear(1), ellipsoid_points_linear(2), ellipsoid_points_linear(3));
h = surf(X,Y,Z);
rotate(h, axiss, rad2deg(angle))
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Ellissoide di Manipolabilità in Velocità (Velocità Lineari)');
axis equal
xlim([-10 10]);
ylim([-10 10])
