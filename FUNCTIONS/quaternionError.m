function eO = quaternionError(desiredQuat, effectiveQuat)
%
% QUATERNIONERROR This function compute the orientation error given two
% quaternion with the scalar component in the end like:
%
%                  quat = [epsX, epsY, epsZ, eta]
%
%   INPUT:
%           1. desiredQuat: Desired quaternion, 4x1 array.
%           2. effectiveQuat: Current quaternion, 4x1 array.
%
%   OUTPUT:
%           1. eO: Orientation errors, 3x1 array
%

    etaE = effectiveQuat(4);
    etaD = desiredQuat(4);

    epsE = effectiveQuat(1:3);
    epsD = desiredQuat(1:3);

    eO = etaE*epsD - etaD*epsE - cross(epsD, epsE);

    deltaEta = etaE*etaD + epsE'*epsD;

    if deltaEta < 0

        eO = -1*eO;

    end

end