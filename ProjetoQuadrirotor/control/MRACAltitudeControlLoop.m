function [altOut, controllerState] = MRACAltitudeControlLoop(state, zDesired, controllerConfig, mracParams, controllerState, quadConfig)
% MRACAltitudeControlLoop
% -------------------------------------------------------------------------
% Controle MRAC de altitude para o modelo MRAC_Test.
%
% A saida adaptativa e tratada como incremento de empuxo em torno do hover:
%   thrust = thrustHover + deltaThrustMRAC
% -------------------------------------------------------------------------

    idx = StateIndex();

    z = state(idx.z);
    zDot = state(idx.zDot);
    phi = state(idx.phi);
    theta = state(idx.theta);

    dtController = controllerConfig.updatePeriod;

    [deltaThrust, controllerState] = MRACSecondOrder( ...
        z, ...
        zDot, ...
        zDesired, ...
        0, ...
        mracParams, ...
        controllerState, ...
        dtController);

    attitudeFactor = cos(phi)*cos(theta);
    attitudeFactor = max(attitudeFactor, 0.20);

    thrustHover = quadConfig.mass*quadConfig.gravity/attitudeFactor;
    thrustRaw = thrustHover + deltaThrust;
    thrust = Saturate(thrustRaw, quadConfig.control.minThrust, quadConfig.control.maxThrust);

    altOut = struct();
    altOut.thrust = thrust;
    altOut.thrustRaw = thrustRaw;
    altOut.deltaThrust = deltaThrust;
    altOut.thrustHover = thrustHover;
    altOut.referenceState = controllerState.xRef;
    altOut.error = controllerState.error;
    altOut.KxHat = controllerState.KxHat;
    altOut.KrHat = controllerState.KrHat;
    altOut.KdHat = controllerState.KdHat;
    altOut.thrustSaturated = thrust ~= thrustRaw;
end
