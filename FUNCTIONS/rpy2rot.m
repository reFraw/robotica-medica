function R = rpy2rot(ANGLES, DEG)
%
% RPY2ROT Calculate the rotation matrix given RPY angles.
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

    firstRow = [cf*ct, cf*st*sp-sf*cp, cf*st*cp+sf*sp];
    secondRow = [sf*ct, sf*st*sp+cf*cp, sf*st*cp-cf*sp];
    thirdRow = [-st, ct*sp, ct*cp];

    R = [firstRow; secondRow; thirdRow];

end