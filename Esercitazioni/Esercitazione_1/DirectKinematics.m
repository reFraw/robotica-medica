function T0 = DirectKinematics(DH)

% This function, from the Denavit-Hartenberg table, calculates a tensor
% containing the homogeneous transformations between the i-th joint and the
% base triad.
% The last matrix of this tensor represents the direct kinematics matrix.

% INPUT : Nx4 matrix representing the complete Denavit-Hartenberg table.

% OUTPUT : Tensor 4x4xN where each 4x4 matrix represents the homogeneous
%          transformation between the i-th triplet and the base triplet.

    nArms = length(DH(:,1));
    T0 = zeros(4, 4, nArms);

    T0(:,:,1) = Homogeneous(DH(1,:));

    for i = 2 : nArms

        T0(:,:,i) = T0(:,:,i-1) * Homogeneous(DH(i,:));

    end

end