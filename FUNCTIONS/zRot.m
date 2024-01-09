function R = zRot(angle)

    c = cos(angle);
    s = sin(angle);

    firstCol = [c s 0]';
    secondCol = [-s c 0]';
    thirdCol = [0 0 1]';

    R = [firstCol secondCol thirdCol];

end