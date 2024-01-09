function R = zyz2rot(ANGLES, DEG)
%
% ZYZ2ROT Calculate the rotation matrix given ZYZ angles.
%   
%   The angles must be given in array format like [phi, theta, psi].
%
%   The unit measure of the angles must be radiants otherwise flag DEG
%   optional parameter to 1.
%

    if ~exist('DEG', 'var')

        DEG = 0;

    end

    if DEG == 1

        ANGLES = deg2rad(ANGLES);

    end

    phi = ANGLES(1);
    theta = ANGLES(2);
    psi = ANGLES(3);

    sf = sin(phi);
    cf = cos(phi);

    st = sin(theta);
    ct = cos(theta);

    sp = sin(psi);
    cp = cos(psi);

    firstRow = [cf*ct*cp-sf*sp, -cf*ct*sp-sf*cp, cf*st];
    secondRow = [sf*ct*cp+cf*sp, -sf*ct*sp+cf*cp, sf*st];
    thirdRow = [-st*cp, st*sp, ct];

    R = [firstRow; secondRow; thirdRow];

end