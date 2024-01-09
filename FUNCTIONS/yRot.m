function R = yRot(angle)

    c = cos(angle);
    s = sin(angle);

    firstCol = [c 0 -s]';
    secondCol = [0 1 0]';
    thirdCol = [s 0 c]';

    R = [firstCol secondCol thirdCol];

end