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