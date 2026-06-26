function [altOut, controllerState] = AltitudeControlLoop(state, ref, controllerConfig, quadConfig, controllerState)
% AltitudeControlLoop
% -------------------------------------------------------------------------
% Controlador classico de altitude usado quando o modelo MRAC_Test nao usa
% MRAC no eixo z. A estrutura replica apenas o canal z do controlador de
% posicao normal.
% -------------------------------------------------------------------------

    if nargin < 5 || isempty(controllerState)
        controllerState.integralError = 0;
    end

    idx = StateIndex();

    z = state(idx.z);
    zDot = state(idx.zDot);

    zError = ref.r(3) - z;
    zDotError = ref.v(3) - zDot;

    dtController = controllerConfig.updatePeriod;
    controlType = upper(string(controllerConfig.type));
    gains = controllerConfig.gains;

    accelerationFeedback = 0;

    switch controlType
        case "P"
            accelerationFeedback = gains.P.Kp(3,3)*zError;

        case "PD"
            accelerationFeedback = gains.PD.Kp(3,3)*zError + ...
                gains.PD.Kd(3,3)*zDotError;

        case "PID"
            controllerState.integralError = controllerState.integralError + dtController*zError;
            if isfield(controllerConfig, 'integralLimit') && ~isempty(controllerConfig.integralLimit)
                lim = controllerConfig.integralLimit;
                if numel(lim) > 1
                    lim = lim(3);
                end
                controllerState.integralError = Saturate(controllerState.integralError, -lim, lim);
            end
            accelerationFeedback = gains.PID.Kp(3,3)*zError + ...
                gains.PID.Kd(3,3)*zDotError + ...
                gains.PID.Ki(3,3)*controllerState.integralError;
    end

    accelerationCommandZ = ref.a(3) + accelerationFeedback;

    thrustRaw = quadConfig.mass*(quadConfig.gravity + accelerationCommandZ);
    thrust = Saturate(thrustRaw, quadConfig.control.minThrust, quadConfig.control.maxThrust);

    altOut = struct();
    altOut.thrust = thrust;
    altOut.thrustRaw = thrustRaw;
    altOut.deltaThrust = thrust - quadConfig.mass*quadConfig.gravity;
    altOut.thrustHover = quadConfig.mass*quadConfig.gravity;
    altOut.referenceState = [ref.r(3); ref.v(3)];
    altOut.error = [z - ref.r(3); zDot - ref.v(3)];
    altOut.KxHat = zeros(2,1);
    altOut.KrHat = 0;
    altOut.KdHat = 0;
    altOut.accelerationCommandZ = accelerationCommandZ;
    altOut.accelerationFeedbackZ = accelerationFeedback;
    altOut.thrustSaturated = thrust ~= thrustRaw;
end
