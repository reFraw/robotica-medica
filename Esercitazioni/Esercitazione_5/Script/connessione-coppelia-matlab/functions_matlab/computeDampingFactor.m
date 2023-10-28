function k = computeDampingFactor(minSVD)

    if minSVD >= 0 && minSVD < 0.15
        k = 1000;
    else
        k = 1000*exp(-minSVD);
    end


end

