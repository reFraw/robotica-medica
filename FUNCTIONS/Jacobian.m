function J = Jacobian(DH, jointConfiguration)
%
% This function calculate the geometrical Jacobian given the Denavit
% Hartenberg table.
%
% INPUT
%       1. DH : nx4 matrix that represents the Denavit Hartenberg table
%       2. jointConfiguration (optional) : n-Dimensional array that represents the
%       joint configuration. (ex. [1, 0, 1] where 1 stand for spherical
%       joints and 0 stands for prismatical joints.
%
% OUTPUT : 6xn matrix that represents the geometrical Jacobian
%


    % Check the number of links
    nLinks = length(DH(:,1));
    
    % if 'jointConfiguration' variable is not given assumes all spherical
    % joint
    if ~exist('jointConfiguration', 'var')
        jointConfiguration = ones(1,nLinks);
    end
    
    % Check if the length of 'jointConfiguration' variable is equal to the
    % number of links in the DH table
    if length(jointConfiguration) ~= length(DH(:,1))
        disp(">>> Invalid joint configuration...")
        
    else
        % Calculate the homogeneous transformation matrix from base
        % triplets to i-th triplets
        T0 = DirectKinematics(DH);

        % extract the (A^0_e) matrix and calculate the position of (O^0_e)
        A0_N = T0(:,:,nLinks);
        pe = A0_N(1:3, 4);
        
        % Set value for p0 and z0 and initialize the Jacobian matrix
        p0 = [0 0 0]';
        z0 = [0 0 1]';
        
        J = zeros(6, nLinks);
        
        % Calculate the first column of the Jacobian matrix
        if jointConfiguration(1) == 1
            firstColumn = [cross(z0, (pe-p0)); z0];
            J(:,1) = firstColumn;

        else
            J(:,1) = [z0' 0 0 0]';

        end
        
        % Calculate the remaining columns of the Jacobian matrix 
        for i = 2 : nLinks
            A = T0(:,:,i-1);
    
            p = A(1:3, 4);
            z = A(1:3, 3);

            if jointConfiguration(i) == 1
                JPi = cross(z, (pe-p));
                JOi = z;
        
                Ji = [JPi; JOi];

            else
                Ji = [z; 0; 0; 0];

            end

            J(:,i) = Ji;
    
        end
     
    end

end