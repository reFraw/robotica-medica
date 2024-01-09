function angles = rot2zyz(R, DOMAIN)
%
% ROT2ZYZ Compute the ZYZ angles given the corripondent rotation matrix.
%
%   The function return the angles in array format like [phi, theta, psi].
%
%   Flag the DOMAIN (Default value 0) optional parameter to 0 if theta in (0,pi) else 1
%   for theta in (-pi, 0)
%

    if ~exist('DOMAIN', 'var')

        DOMAIN = 0;

    end

    r23 = R(2,3);
    r13 = R(1,3);
    r33 = R(3,3);
    r32 = R(3,2);
    r31 = R(3,1);
    
    if DOMAIN == 0
        phi = atan2(r23, r13);
        theta = atan2(sqrt(r13^2 + r23^2), r33);
        psi = atan2(r32, -r31);

    else
        phi = atan2(-r23, -r13);
        theta = atan2(-sqrt(r13^2 + r23^2), r33);
        psi = atan2(-r32, r31);

    end

    angles = [phi, theta, psi]';

end