function eO = quatError(quatE, quatD)
    
    etaE = quatE(1);
    etaD = quatD(1);

    epsE = quatE(2:end);
    epsD = quatD(2:end);

    eO = etaE*epsD - etaD*epsE - cross(epsD, epsE);

    eta_tilde = quatE(1)*quatD(1) + quatE(2:end)'*quatD(2:end);

    if eta_tilde < 0

        eO = -eO;

    end

end

