function [currentPos, currentVel, currentAcc] = trapezoidal( ...
    qIn, ...
    qOut, ...
    finalTime, ...
    cruiseVelocity, ...
    currentTime)

nLink = length(qIn);

currentPos = zeros(nLink, 1);
currentVel = zeros(nLink, 1);
currentAcc = zeros(nLink, 1);

for i = 1 : nLink

    if ~checkCruiseVelocity(cruiseVelocity, qIn(i), qOut(i), finalTime)

        cruiseVelocity = 3*abs(qOut(i) - qIn(i))/(2*finalTime);

    end

    tc = (qIn(i) - qOut(i) + cruiseVelocity*finalTime)/cruiseVelocity;
    qDdot = cruiseVelocity^2/(qIn(i) - qOut(i) + cruiseVelocity*finalTime);

    currentPos(i) = position( ...
        qIn(i), ...
        qOut(i), ...
        finalTime, ...
        tc, ...
        qDdot, ...
        currentTime);

    currentVel(i) = velocity( ...
        finalTime, ...
        tc, ...
        qDdot, ...
        currentTime);

    currentAcc(i) = acceleration( ...
        finalTime, ...
        tc, ...
        qDdot, ...
        currentTime);

end

end


function check = checkCruiseVelocity( ...
    cruiseVelocity, ...
    qIn, ...
    qOut, ...
    finalTime)

checkTerm = abs(qOut - qIn)/finalTime;

check = (abs(cruiseVelocity) > checkTerm) && (abs(cruiseVelocity) <= 2*checkTerm); 

end

function pos = position( ...
    qIn, ...
    qOut, ...
    finalTime, ...
    tc, ...
    qDdot, ...
    currentTime)
    
    if currentTime >= 0 && currentTime <= tc

        pos = qIn + 0.5*qDdot*currentTime^2;

    elseif currentTime > tc && currentTime <= finalTime - tc

        pos = qIn + qDdot*tc*(currentTime -0.5*tc);

    elseif currentTime > finalTime - tc && currentTime < finalTime

        pos = qOut - 0.5*qDdot*(finalTime - currentTime)^2;

    else
        
        pos = qOut;

    end

end

function vel = velocity( ...
    finalTime, ...
    tc, ...
    qDdot, ...
    currentTime)
    
    if currentTime >= 0 && currentTime <= tc

        vel = qDdot*currentTime;

    elseif currentTime > tc && currentTime <= finalTime - tc

        vel = qDdot*tc;

    elseif currentTime > finalTime - tc && currentTime < finalTime

        vel = qDdot*finalTime - qDdot*currentTime;

    else 

        vel = 0;

    end

end

function acc = acceleration( ...
    finalTime, ...
    tc, ...
    qDdot, ...
    currentTime)
    
    if currentTime >= 0 && currentTime <= tc

        acc = qDdot;

    elseif currentTime > tc && currentTime <= finalTime - tc

        acc = 0;

    elseif currentTime > finalTime - tc && currentTime < finalTime

        acc = -qDdot;

    else

        acc = 0;

    end

end
