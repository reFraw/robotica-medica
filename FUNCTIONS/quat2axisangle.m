function [axis, angle] = quat2axisangle(QUAT, DEG)
%
% QUAT2AXISANGLE Compute the axis-angle representation associated with the
% given quaternion.
%
%   This function returns the axis-angle representation using two output
%   variables.
%   AXIS is a 3x1 array built like [rX, rY, rZ].
%   ANGLE is a scalar expressed in radiants. If you want to use deg instead
%   flag the DEG optional parameters to 1.
%

    if ~exist('DEG', 'var')

        DEG = 0;

    end

    eta = QUAT(4);

    eps = QUAT(1:3);

    angle = 2*acos(eta);

    axis = eps ./ sin(angle/2);

    if DEG == 1

        angle = rad2deg(angle);

    end

end