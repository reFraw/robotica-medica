function T0 = DirectKinematics(DH)

    nArms = length(DH(:,1));
    T0 = zeros(4, 4, nArms);

    T0(:,:,1) = Homogeneous(DH(1,:));

    for i = 2 : nArms

        T0(:,:,i) = T0(:,:,i-1) * Homogeneous(DH(i,:));

    end

end