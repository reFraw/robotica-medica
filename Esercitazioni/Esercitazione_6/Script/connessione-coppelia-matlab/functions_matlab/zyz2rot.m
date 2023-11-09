function rotMatrix = zyz2rot(phi, theta, psi)
    
    cf = cos(phi);
    sf = sin(phi);

    ct = cos(theta);
    st = sin(theta);

    cp = cos(psi);
    sp = sin(psi);

    firstRow = [cf*ct*cp - sf*sp, -cf*ct*sp - sf*cp, cf*st];
    secondRow = [sf*ct*cp + cf*sp, -sf*ct*sp + cf*cp, sf*st];
    thirdRow = [-st*cp, st*sp, ct];

    rotMatrix = [firstRow; secondRow; thirdRow];

end

