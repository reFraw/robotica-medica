function MSD = minSingularValue(J)

    S = svd(J);
    MSD = min(S);

end