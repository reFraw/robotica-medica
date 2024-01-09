function R = axisangle2rot(AXIS, ANGLE, DEG)
%
% AXISANGLE2ROT(AXIS, ANGLE, DEG) Compute the rotation matrix given the axis-angle
% representation.
%
% The measure unit of the angle must be radiants but if you want to use deg
% you must flag DEG optional parameters to 1.
%

    if ~exist('DEG', 'var')

        DEG = 0;

    end

    if DEG == 1

        ANGLE = deg2rad(ANGLE);

    end

    rx = AXIS(1);
    ry = AXIS(2);
    rz = AXIS(3);

    ct = cos(ANGLE);
    st = sin(ANGLE);

    R = [rx^2*(1-ct)+ct rx*ry*(1-ct)-rz*st rx*rz*(1-ct)+ry*st;
     rx*ry*(1-ct)+rz*st ry^2*(1-ct)+ct ry*rz*(1-ct)-rx*st;
     rx*rz*(1-ct)-ry*st ry*rz*(1-ct)+rx*st rz^2*(1-ct)+ct];

end