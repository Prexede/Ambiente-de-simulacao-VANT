function [posOut, controllerState] = PositionControlLoop(state, ref, controllerConfig, quadConfig, controllerState)
% PositionControlLoop
% -------------------------------------------------------------------------
% Controlador unico de posicao.
% Usa posicao, velocidade e aceleracao desejadas da trajetoria.
% -------------------------------------------------------------------------

    if nargin < 5 || isempty(controllerState)
        controllerState.integralError = zeros(3,1);
    end

    idx = StateIndex();
    position = state(idx.position);
    velocity = state(idx.velocity);

    positionError = ref.r - position;
    velocityError = ref.v - velocity;

    dtController = controllerConfig.updatePeriod;
    controlType = upper(string(controllerConfig.type));
    gains = controllerConfig.gains;

    switch controlType
        case "P"
            accelerationFeedback = PController(positionError, gains.P);

        case "PD"
            accelerationFeedback = PDController(positionError, velocityError, gains.PD);

        case "PID"
            controllerState.integralError = controllerState.integralError + dtController*positionError;
            if isfield(controllerConfig, 'integralLimit') && ~isempty(controllerConfig.integralLimit)
                lim = controllerConfig.integralLimit;
                controllerState.integralError = Saturate(controllerState.integralError, -lim, lim);
            end
            accelerationFeedback = PIDController(positionError, velocityError, controllerState.integralError, gains.PID);

        otherwise
            error('Tipo de controlador de posicao invalido.');
    end

    accelerationCommand = ref.a + accelerationFeedback;

    mass = quadConfig.mass;
    gravity = quadConfig.gravity;

    thrustCommandRaw = mass*(gravity + accelerationCommand(3));
    thrustCommand = Saturate(thrustCommandRaw, quadConfig.control.minThrust, quadConfig.control.maxThrust);

    psiDesired = ref.psi;

    % Relacao coerente com R = Rz(psi)*Ry(theta)*Rx(phi):
    % para psi = 0, x_ddot ~= g*theta e y_ddot ~= -g*phi.
    thetaDesiredRaw = ( cos(psiDesired)*accelerationCommand(1) + sin(psiDesired)*accelerationCommand(2) )/gravity;
    phiDesiredRaw   = ( sin(psiDesired)*accelerationCommand(1) - cos(psiDesired)*accelerationCommand(2) )/gravity;

    maxTilt = quadConfig.control.maxTiltAngle;
    phiDesired = Saturate(phiDesiredRaw, -maxTilt, maxTilt);
    thetaDesired = Saturate(thetaDesiredRaw, -maxTilt, maxTilt);

    attitudeDesired = [
        phiDesired;
        thetaDesired;
        psiDesired
    ];

    posOut = struct();
    posOut.thrust = thrustCommand;
    posOut.thrustRaw = thrustCommandRaw;
    posOut.attitudeDesired = attitudeDesired;
    posOut.accelerationCommand = accelerationCommand;
    posOut.accelerationFeedback = accelerationFeedback;
    posOut.positionError = positionError;
    posOut.velocityError = velocityError;
    posOut.integralError = controllerState.integralError;
    posOut.tiltSaturated = [phiDesired ~= phiDesiredRaw; thetaDesired ~= thetaDesiredRaw];
    posOut.thrustSaturated = thrustCommand ~= thrustCommandRaw;
end
