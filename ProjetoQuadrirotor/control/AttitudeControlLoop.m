function [attOut, controllerState] = AttitudeControlLoop(state, attitudeDesired, controllerConfig, controllerState)
% AttitudeControlLoop
% -------------------------------------------------------------------------
% Controlador unico de atitude.
% -------------------------------------------------------------------------

    if nargin < 4 || isempty(controllerState)
        controllerState.integralError = zeros(3,1);
    end

    idx = StateIndex();
    attitude = state(idx.attitude);
    bodyRate = state(idx.bodyRate);

    attitudeError = WrapAngle(attitudeDesired - attitude);
    rateDesired = zeros(3,1);
    rateError = rateDesired - bodyRate;

    dtController = controllerConfig.updatePeriod;
    controlType = upper(string(controllerConfig.type));
    gains = controllerConfig.gains;

    switch controlType
        case "P"
            torqueCommand = PController(attitudeError, gains.P);

        case "PD"
            torqueCommand = PDController(attitudeError, rateError, gains.PD);

        case "PID"
            controllerState.integralError = controllerState.integralError + dtController*attitudeError;
            if isfield(controllerConfig, 'integralLimit') && ~isempty(controllerConfig.integralLimit)
                lim = controllerConfig.integralLimit;
                controllerState.integralError = Saturate(controllerState.integralError, -lim, lim);
            end
            torqueCommand = PIDController(attitudeError, rateError, controllerState.integralError, gains.PID);

        otherwise
            error('Tipo de controlador de atitude invalido.');
    end

    attOut = struct();
    attOut.torque = torqueCommand;
    attOut.attitudeError = attitudeError;
    attOut.rateError = rateError;
    attOut.integralError = controllerState.integralError;
    attOut.attitudeDesired = attitudeDesired;
end
