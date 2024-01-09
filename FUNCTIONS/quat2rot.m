function R = quat2rot(QUAT)
%
% QUAT2ROT Compute the rotation matrix associated with the given
% quaternion.
%
%   Quaternion must be given in 4x1 array format like [eta, epsX, epsY,
%   epsZ].
%

    eta = QUAT(4);

    epsX = QUAT(1);
    epsY = QUAT(2);
    epsZ = QUAT(3);

    R11 = 2*(eta^2 + epsX^2) - 1;
    R12 = 2*(epsX*epsY - eta*epsZ);
    R13 = 2*(epsX*epsZ + eta*epsY);

    R21 = 2*(epsX*epsY + eta*epsZ);
    R22 = 2*(eta^2 + epsY^2) - 1;
    R23 = 2*(epsY*epsZ - eta*epsX);

    R31 = 2*(epsX*epsZ - eta*epsY);
    R32 = 2*(epsY*epsZ + eta*epsX);
    R33 = 2*(eta^2 + epsZ^2) - 1;

    R = [
        R11 R12 R13;
        R21 R22 R23;
        R31 R32 R33
        ];

end