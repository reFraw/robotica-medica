function angles = rot2rpy(R, DOMAIN)
%
% ROT2RPY Compute the RPY angles given the corripondent rotation matrix.
%
%   The function return the angles in array format like [phi, theta, psi].
%
%   Flag the DOMAIN (Default value 0) optional parameter to 0 if theta in (-pi/2,pi/2) else 1
%   for theta in (pi/2, 3pi/2)
%

    if ~exist('DOMAIN', 'var')

        DOMAIN = 0;

    end

    r21 = R(2,1);
    r11 = R(1,1);
    r31 = R(3,1);
    r32 = R(3,2);
    r33 = R(3,3);
    
    if DOMAIN == 0
        phi = atan2(r21, r11);
        theta = atan2(-r31, sqrt(r32^2 + r33^2));
        psi = atan2(r32, r33);

    else
        phi = atan2(-r21, -r11);
        theta = atan2(-r31, -sqrt(r32^2 + r33^2));
        psi = atan2(-r32, -r33);

    end

    angles = [phi theta psi]';

end