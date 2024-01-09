function quat = axisangle2quat(AXIS, ANGLE, DEG)
%
% AXISANGLE2QUAT Compute the quaternion associated with the given
% axis-angle representation.
%
%   AXIS must be a 3x1 array.
%
%   The measure unit of ANGLE must be rad. If you want to use deg instead
%   flag the DEG optional parameter to 1.
%
%   The function returns the quaterion in array format like [eta, epsX,
%   epsY, epsZ].
%

    if ~exist('DEG', 'var')

        DEG = 0;

    end

    if DEG == 1

        ANGLE = deg2rad(ANGLE);

    end

    eta = cos(0.5*ANGLE);
    eps = sin(0.5*ANGLE)*AXIS;

    quat = [eps; eta];

end