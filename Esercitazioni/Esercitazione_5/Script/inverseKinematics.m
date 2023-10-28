function [q, dq, positionErrors, orientationErrors, spacePath, DHS] = inverseKinematics( ...
    setPoint, ...
    desiredOrientation, ...
    startConfiguration, ...
    simulationTime, ...
    positionGain, ...
    orientationGain, ...
    algorithmType)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % This function calculates the values of the joint variables to be
    % assigned to achieve a desired set point at a given orientation.
    % This function is optimized for Kinova Jaco2 7-DOF mainpulator.

    % INPUT
    %
    %    - setPoint : 3x1 array in format [x,y,z]
    %
    %    - desiredOrientation : 3x1 array with ZYZ convention
    %
    %    - startConfiguration : 7x1 array (Theta angles of Jaco2 spherical
    %                           joint)
    %
    %    - simulationTime : Scalar value in seconds, default 60 s.
    %
    %    - positionGain : Scalar value for position gain matrix, default 5.
    %
    %    - orientationGain : Scalar value for orientation gain matrix,
    %                        default 5
    %
    %    - algorithmType : String between "pinv" or "transp", default 
    %                      "pinv".

    % OUTPUT
    %
    %    - q : Matrix containing the values of the joint variables for each
    %          iteration.
    %
    %    - dq : Matrix containing the values of angular velocity for each
    %           iteration.
    %
    %    - positionErrors : Matrix containing the position error. Each
    %                       column of the matrix is formed
    %                       as [ex, ey, ez, t].
    %
    %    - orientationErrors : Matrix containing the orientation error. 
    %                          Each column of the matrix is formed as
    %                          [e_phi, e_theta, e_psi, t].
    %
    %    - spacePath : Matrix containing the trajectory on the operative
    %                  space. Each column of the matrix is formed
    %                  as [x, y, z].
    %
    %    - DHS : 7x4x2 tensor with start and ending DH table.

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

    % Checking optional variables
    if ~exist('simulationTime', 'var')
        simulationTime = 60;
    end

    if ~exist('positionGain', 'var')
        positionGain = 5;
    end

    if ~exist('orientationGain', 'var')
        orientationGain = 5;
    end

    if ~exist('algorithmType', 'var') || algorithmType ~= "pinv" || algorithmType ~= "transp"
        algorithmType = "pinv";
    end
    
    % Denavit-Hartenberg table
    a = zeros(7,1);
    alfa = zeros(7,1);
    alfa(1:6) = pi/2;
    d = [0.2755 0 -0.41 -0.0098 -0.3072 0 0.25]';

    startConfiguration = checkDimension(startConfiguration);

    DH = [a alfa d startConfiguration];
    
    % Time parameters [s]
    samplingTime = 0.01;
    t = 0 : samplingTime : simulationTime;
    nPoints = length(t);
    
    % Gain matrices
    KP = diag(positionGain*ones(1,3));
    KO = diag(orientationGain*ones(1,3));
    
    % Output variables
    q = zeros(7, nPoints);
    dq = zeros(7, nPoints);
    orientationErrors = zeros(4, nPoints);
    orientationErrors(4,:) = t;
    positionErrors = zeros(4, nPoints);
    positionErrors(4,:) = t;
    spacePath = zeros(3, nPoints);
    DHS = zeros(7, 4, 2);

    % Set point parameters
    pD = setPoint;
    RD = zyz2rot( ...
        desiredOrientation(1), ...
        desiredOrientation(2), ...
        desiredOrientation(3));
    QD = rot2quat(RD);

    DHS(:,:,1) = DH;

    for i = 1 : nPoints

        T = DirectKinematics(DH);
        T = T(:,:,end);

        pE = T(1:3, 4);
        spacePath(:,i) = pE;
        RE = T(1:3, 1:3);
        QE = rot2quat(RE);

        ePi = pD - pE;
        eOi = quatError(QE, QD);
        positionErrors(1:3,i) = ePi;
        orientationErrors(1:3,i) = eOi;

        q(:,i) = DH(:,4);

        amplifiedErrors = [KP*ePi; KO*eOi];

        J = Jacobian(DH);

        if algorithmType == "pinv"

            pinvJ = pinv_damped(J);
            dqi = pinvJ*amplifiedErrors;

        else

            transpJ = J';
            dqi = transpJ*amplifiedErrors;

        end

        dq(:,i) = dqi;
        
        qNext = q(:,i) + samplingTime*dqi;
        DH(:,4) = qNext;

    end

    DHS(:,:,end) = DH;

end

%%%%%%%%%%%%%%%%%%%%%%%%%% AUXILIARY FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%                

function check = checkDimension(vector)

    dimension = size(vector);

    if dimension(2) ~= 1
        check = vector';

    else
        check = vector;

    end

end



function R = zyz2rot(phi, theta, psi)

    cf = cos(phi);
    sf = sin(phi);

    ct = cos(theta);
    st = sin(theta);

    cp = cos(psi);
    sp = sin(psi);

    firstRow = [cf*ct*cp - sf*sp, -cf*ct*sp - sf*cp, cf*st];
    secondRow = [sf*ct*cp + cf*sp, -sf*ct*sp + cf*cp, sf*st];
    thirdRow = [-st*cp, st*sp, ct];

    R = [firstRow; secondRow; thirdRow];

end



function Q = rot2quat(R)

    r11 = R(1,1);
    r22 = R(2,2);
    r33 = R(3,3);

    r13 = R(1,3);
    r31 = R(3,1);

    r23 = R(2,3);
    r32 = R(3,2);

    r12 = R(1,2);
    r21 = R(2,1);

    eta = 0.5*sqrt(r11 + r22 + r33 + 1);

    eps1 = 0.5*sign(r32 - r23)*sqrt(r11 - r22 - r33 + 1);
    eps2 = 0.5*sign(r13 - r31)*sqrt(r22 - r33 - r11 + 1);
    eps3 = 0.5*sign(r21 - r12)*sqrt(r33 - r11 - r22 + 1);

    Q = [eta eps1 eps2 eps3]';

end



function eO = quatError(QE, QD)

    etaE = QE(1);
    etaD = QD(1);

    epsE = QE(2:end);
    epsD = QD(2:end);

    eO = etaE*epsD - etaD*epsE - cross(epsD, epsE);

    eta_tilde = QE(1)*QD(1) + QE(2:end)'*QD(2:end);

    if eta_tilde < 0

        eO = -eO;

    end

end



function T = Homogeneous(DH_row)

   a = DH_row(1);
   alfa = DH_row(2);
   d = DH_row(3);
   theta = DH_row(4);

   ca = cos(alfa);
   sa = sin(alfa);

   ct = cos(theta);
   st = sin(theta);

   firstColumn = [
       ct,
       st,
       0,
       0];

   secondColumn = [
       -st*ca,
       ct*ca,
       sa,
       0];

   thirdColumn = [
       st*sa,
       -ct*sa,
       ca,
       0];

   fourthColumn = [
       a*ct,
       a*st,
       d,
       1];

   T = [firstColumn secondColumn thirdColumn fourthColumn];

end



function T0 = DirectKinematics(DH)

    nArms = length(DH(:,1));
    T0 = zeros(4, 4, nArms);

    T0(:,:,1) = Homogeneous(DH(1,:));

    for i = 2 : nArms

        T0(:,:,i) = T0(:,:,i-1) * Homogeneous(DH(i,:));

    end

end



function J = Jacobian(DH, jointConfiguration)

    % This function calculate the geometrical Jacobian given the Denavit
    % Hartenberg table.

    % INPUT 
    %       1. DH : nx4 matrix that represents the Denavit Hartenberg table
    %       2. jointConfiguration (optional) : n-Dimensional array that represents the
    %       joint configuration. (ex. [1, 0, 1] where 1 stand for spherical
    %       joints and 0 stands for prismatical joints.

    % OUTPUT : 6xn matrix that represents the geometrical Jacobian



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



function pinvJ = pinv_damped(J)

[U,S,V] = svd(J);

threshold = 0.01;
lambda = 0.01;

for i=1:size(J,1)
    eigval(i) = S(i,i);
end

Sinv = zeros(size(J,2), size(J,1));

for i=1:size(J,1)

    p(i) = cosRialzato(eigval(i), lambda, threshold);
    Sinv(i,i) = eigval(i)/(eigval(i)^2 + p(i));
   
    
end

pinvJ = V * Sinv * U';

end



function reg = cosRialzato(sigma, lambda, threshold)

    sigma = abs(sigma);
    
    if sigma < threshold
        
        tmp = (sigma/threshold)* pi;
        reg = lambda * (0.5 * cos(tmp) + 0.5);

    else
        reg = 0;
        
    end

end