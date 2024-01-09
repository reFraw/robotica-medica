function R = xRot(angle)

    c = cos(angle);
    s = sin(angle);

    firstCol = [1 0 0]';
    secondCol = [0 c s]';
    thirdCol = [0 -s c]';

    R = [firstCol secondCol thirdCol];

end