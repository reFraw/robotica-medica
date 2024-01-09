function quat = rot2quat(R)
%
% ROT2QUAT Compute the quaternion associated with the given rotation
% matrix.
%
%   The function return quaternion in 4x1 array format like [eta, epsX,
%   epsY, epsZ].
%
    
    r11 = R(1,1);
    r12 = R(1,2);
    r13 = R(1,3);

    r21 = R(2,1);
    r22 = R(2,2);
    r23 = R(2,3);

    r31 = R(3,1);
    r32 = R(3,2);
    r33 = R(3,3);

    quat = zeros(4,1);

    quat(4) = sqrt(r11 + r22 + r33 + 1) / 2;

    if r32 - r23 >= 0
        quat(1) = .5*sqrt(r11 - r22 - r33 + 1);
    else
        quat(1) = -.5*sqrt(r11 - r22 - r33 + 1);
    end

    if r13 - r31 >= 0
        quat(2) = .5*sqrt(r22 - r33 - r11 + 1);
    else
        quat(2) = -.5*sqrt(r22 - r33 - r11 + 1);
    end

    if r21 - r12 >= 0
        quat(3) = .5*sqrt(r33 - r11 - r22 + 1);
    else
        quat(3) = -.5*sqrt(r33 - r11 - r22 + 1);
    end
    
    quat = real(quat);
    quat = quat / norm(quat);

end