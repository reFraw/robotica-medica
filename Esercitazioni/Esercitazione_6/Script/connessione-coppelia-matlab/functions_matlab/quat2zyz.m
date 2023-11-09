function [phi, theta, psi] = quat2zyz(Q)

    eta = Q(1);
    qx = Q(2);
    qy = Q(3);
    qz = Q(4);

    phi = atan2(2*(eta*qx+qy*qz), 1-2*(qx^2+qy^2));
    theta = atan2(sqrt(1+2*(eta*qy-qx*qz)), sqrt(1-2*(eta*qy-qx*qz)));
    psi = atan2(2*(eta*qz+qx*qy), 1-2*(qy^2+qz^2));
    
end

