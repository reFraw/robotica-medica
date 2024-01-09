function [currentPosition, currentVelocity, currentAcceleration] = trapezoidal( ...
    startPoint, ...
    endPoint, ...
    finalTime, ...
    cruiseVelocity, ...
    currentTime)
%
% TRAPEZOIDAL This function compute current position, velocity and
% accelaration with trapezoidal speed profile.
%

    numComponents = length(startPoint);

    % Verifico se il tempo finale è comune a tutte le componenti
    if length(finalTime) == 1 && numComponents ~= 1

        finalTime = finalTime*ones(numComponents,1);

    end

    % Verifico se la velocità di crociera è comune a tutte le componenti
    if length(cruiseVelocity) == 1 && numComponents ~= 1

        cruiseVelocity = cruiseVelocity*ones(numComponents,1);

    end


    % Check della velocità di crociera inserita. Se non soddisfa la condizione
    % viene imposta una velocità di crociera default.

    
    cruiseVelocities = zeros(numComponents,1);

    currentPosition = zeros(numComponents,1);
    currentVelocity = zeros(numComponents,1);
    currentAcceleration = zeros(numComponents,1);
    
    for i = 1 : numComponents

        qDotC_i = cruiseVelocity(i);
        checkTerm = abs(endPoint(i)-startPoint(i))/finalTime(i);
        
        if ~((abs(qDotC_i)>checkTerm) && (abs(qDotC_i)<=2*checkTerm))

            newCruiseVel = 1.5*checkTerm;
            cruiseVelocities(i) = newCruiseVel;

        else

            cruiseVelocities(i) = qDotC_i;

        end

    end

    % Per ogni componente calcolo posizione, velocità e accelerazione
    for i = 1 : numComponents

        commonTerm = startPoint(i) - endPoint(i) + cruiseVelocities(i)*finalTime(i);

        tc = commonTerm/cruiseVelocities(i);
        ddotqc = cruiseVelocities(i)^2/commonTerm;

        currentPosition(i) = computePosition( ...
            startPoint(i), ...
            endPoint(i), ...
            currentTime, ...
            tc, ...
            finalTime(i), ...
            ddotqc);

        currentVelocity(i) = computeVelocity( ...
            currentTime, ...
            tc, ...
            finalTime(i), ...
            ddotqc);

        currentAcceleration(i) = computeAcceleration( ...
            currentTime, ...
            tc, ...
            finalTime(i), ...
            ddotqc);

    end

end

function pos = computePosition(qi, qf, t, tc, tf, ddotqc)

    if t >= 0 && t <= tc
        pos = qi + 0.5*ddotqc*t^2;

    elseif t > tc && t <= tf-tc
        pos = qi + ddotqc*tc*(t-0.5*tc);

    elseif t > tf-tc && t <= tf
        pos = qf - 0.5*ddotqc*(tf-t)^2;

    else
        pos = qf;

    end

end

function vel = computeVelocity(t, tc, tf, ddotqc)

    if t >= 0 && t <= tc
        vel = ddotqc*t;
    
    elseif t > tc && t <= tf-tc
        vel = ddotqc*tc;

    elseif t > tf-tc && t <= tf
        vel = ddotqc*(tf-t);

    else
        vel = 0;

    end

end

function acc = computeAcceleration(t, tc, tf, ddotqc)

    if t >= 0 && t <= tc
        acc = ddotqc;

    elseif t > tc && t <= tf-tc
        acc = 0;

    elseif t > tf-tc && t <= tf
        acc = -ddotqc;

    else
        acc = 0;

    end

end