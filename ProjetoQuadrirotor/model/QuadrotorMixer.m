function mixerOut = QuadrotorMixer(thrustCommand, torqueCommand, quadConfig)
% QuadrotorMixer
% -------------------------------------------------------------------------
% Converte empuxo total e torques desejados em velocidades dos motores.
% Os limites dos motores sao lidos de quadConfig.actuator.
% -------------------------------------------------------------------------

    torqueCommand = torqueCommand(:);
    if numel(torqueCommand) ~= 3
        error('torqueCommand deve ser um vetor 3x1.');
    end

    kThrust = quadConfig.kThrust;
    kDrag = quadConfig.kDrag;
    armLength = quadConfig.armLength;

    omegaMin = quadConfig.actuator.omegaMin;
    omegaMax = quadConfig.actuator.omegaMax;

    omegaSquaredMin = omegaMin^2;
    omegaSquaredMax = omegaMax^2;

    commandDesired = [
        thrustCommand;
        torqueCommand(1);
        torqueCommand(2);
        torqueCommand(3)
    ];

    mixerMatrix = [
        kThrust,             kThrust,             kThrust,             kThrust;
        0,                  -armLength*kThrust,  0,                   armLength*kThrust;
        armLength*kThrust,   0,                  -armLength*kThrust,   0;
       -kDrag,               kDrag,              -kDrag,               kDrag
    ];

    omegaSquaredRaw = mixerMatrix \ commandDesired;
    omegaSquared = Saturate(omegaSquaredRaw, omegaSquaredMin, omegaSquaredMax);
    motorOmega = sqrt(omegaSquared);
    commandActual = mixerMatrix*omegaSquared;

    mixerOut = struct();
    mixerOut.motorOmega = motorOmega;
    mixerOut.omegaSquared = omegaSquared;
    mixerOut.omegaSquaredRaw = omegaSquaredRaw;
    mixerOut.commandDesired = commandDesired;
    mixerOut.commandActual = commandActual;
    mixerOut.saturated = abs(omegaSquared - omegaSquaredRaw) > 1e-9;
    mixerOut.anySaturated = any(mixerOut.saturated);
    mixerOut.mixerMatrix = mixerMatrix;
end
