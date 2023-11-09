function Q = rot2quat(R)

    r11 = R(1,1);
    r22 = R(2,2);
    r33 = R(3,3);

    r13 = R(1,3);
    r31 = R(3,1);

    r23 = R(2,3);
    r32 = R(3,2);

    r12 = R(1,2);
    r21 = R(2,1);

    eta = 0.5*sqrt(r11 + r22 + r33 + 1);

    eps1 = 0.5*sign(r32 - r23)*sqrt(r11 - r22 - r33 + 1);
    eps2 = 0.5*sign(r13 - r31)*sqrt(r22 - r33 - r11 + 1);
    eps3 = 0.5*sign(r21 - r12)*sqrt(r33 - r11 - r22 + 1);

    Q = [eta eps1 eps2 eps3]';

end