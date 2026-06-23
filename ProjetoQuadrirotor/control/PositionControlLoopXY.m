function [posOut, controllerState] = PositionControlLoopXY(state, ref, controllerConfig, quadConfig, controllerState)
% PositionControlLoopXY
% -------------------------------------------------------------------------
% Controlador horizontal usado no modelo MRAC_Test.
%
% Esta funcao usa x/y para gerar phi_des e theta_des. A altitude z nao e
% controlada aqui, pois o empuxo fica com o MRACAltitudeControlLoop.
% -------------------------------------------------------------------------

    if nargin < 5 || isempty(controllerState)
        controllerState.integralError = zeros(3,1);
    end

    idx = StateIndex();
    position = state(idx.position);
    velocity = state(idx.velocity);

    positionError = ref.r - position;
    velocityError = ref.v - velocity;

    positionErrorForControl = [positionError(1); positionError(2); 0];
    velocityErrorForControl = [velocityError(1); velocityError(2); 0];

    dtController = controllerConfig.updatePeriod;
    controlType = upper(string(controllerConfig.type));
    gains = controllerConfig.gains;

    switch controlType
        case "P"
            accelerationFeedback = PController(positionErrorForControl, gains.P);

        case "PD"
            accelerationFeedback = PDController(positionErrorForControl, velocityErrorForControl, gains.PD);

        case "PID"
            controllerState.integralError = controllerState.integralError + dtController*positionErrorForControl;
            if isfield(controllerConfig, 'integralLimit') && ~isempty(controllerConfig.integralLimit)
                lim = controllerConfig.integralLimit;
                controllerState.integralError = Saturate(controllerState.integralError, -lim, lim);
            end
            accelerationFeedback = PIDController(positionErrorForControl, velocityErrorForControl, controllerState.integralError, gains.PID);
    end

    accelerationCommand = [ref.a(1); ref.a(2); 0] + accelerationFeedback;

    gravity = quadConfig.gravity;
    psiDesired = ref.psi;

    thetaDesiredRaw = ( cos(psiDesired)*accelerationCommand(1) + sin(psiDesired)*accelerationCommand(2) )/gravity;
    phiDesiredRaw   = ( sin(psiDesired)*accelerationCommand(1) - cos(psiDesired)*accelerationCommand(2) )/gravity;

    maxTilt = quadConfig.control.maxTiltAngle;
    phiDesired = Saturate(phiDesiredRaw, -maxTilt, maxTilt);
    thetaDesired = Saturate(thetaDesiredRaw, -maxTilt, maxTilt);

    attitudeDesired = [phiDesired; thetaDesired; psiDesired];

    posOut = struct();
    posOut.attitudeDesired = attitudeDesired;
    posOut.accelerationCommand = accelerationCommand;
    posOut.accelerationFeedback = accelerationFeedback;
    posOut.positionError = positionError;
    posOut.velocityError = velocityError;
    posOut.integralError = controllerState.integralError;
    posOut.tiltSaturated = [phiDesired ~= phiDesiredRaw; thetaDesired ~= thetaDesiredRaw];
end
