function [axis, angle] = rot2axisangle(R, DEG)
%
% ROT2AXISANGLE Compute the axis-angle representation given the
% corrispondent rotation maxtrix.
%
%   The function returns the axis-angle in array format like [axis, angle],
%   where axis is a 3x1 array built like [rX, rY, rZ]'.
%
%   The measure unit of angle is rad. If you want deg flag the DEG optional
%   parameters to 1.
%

    if ~exist('DEG', 'var')

        DEG = 0;

    end

    r11 = R(1,1);
    r22 = R(2,2);
    r33 = R(3,3);
    r32 = R(3,2);
    r23 = R(2,3);
    r13 = R(1,3);
    r31 = R(3,1);
    r21 = R(2,1);
    r12 = R(1,2);

    angle = acos((r11+r22+r33-1)/2);

    rX = r32 - r23;
    rY = r13 - r31;
    rZ = r21 - r12;

    axis = (1/(2*sin(angle)))*[rX rY rZ]';

    if DEG == 1

        angle = rad2deg(angle);

    end

end