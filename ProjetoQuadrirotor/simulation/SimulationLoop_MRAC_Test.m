function simData = SimulationLoop_MRAC_Test(simConfig, quadConfig, controllersConfig, mracParams, flightPlan, noiseParams)
% SimulationLoop_MRAC_Test
% -------------------------------------------------------------------------
% Loop de simulacao para o modelo MRAC_Test.
%
% Estrutura:
%   x/y -> PositionControlLoopXY -> phi_des/theta_des
%   z   -> MRACAltitudeControlLoop -> thrust
%   atitude -> MRACAttitudeControlLoop -> torques
% -------------------------------------------------------------------------

    idx = StateIndex();

    t = simConfig.time.t(:).';
    Ts = simConfig.time.Ts;
    N = numel(t);

    positionStep = max(1, round(1/(controllersConfig.position.updateFrequency*Ts)));
    altitudeStep = max(1, round(1/(controllersConfig.altitude.updateFrequency*Ts)));
    attitudeStep = max(1, round(1/(controllersConfig.attitude.updateFrequency*Ts)));

    controllersConfig.position.updateStep = positionStep;
    controllersConfig.position.actualFrequency = 1/(positionStep*Ts);
    controllersConfig.position.updatePeriod = positionStep*Ts;
    controllersConfig.position.integralLimit = inf(3,1);

    controllersConfig.altitude.updateStep = altitudeStep;
    controllersConfig.altitude.actualFrequency = 1/(altitudeStep*Ts);
    controllersConfig.altitude.updatePeriod = altitudeStep*Ts;

    controllersConfig.attitude.updateStep = attitudeStep;
    controllersConfig.attitude.actualFrequency = 1/(attitudeStep*Ts);
    controllersConfig.attitude.updatePeriod = attitudeStep*Ts;

    simData = InitializeSimulationData( ...
        simConfig, ...
        quadConfig, ...
        controllersConfig, ...
        flightPlan);

    simData.state.xMeasured = nan(12, N);
    simData.config.noise = noiseParams;
    simData.config.mrac = mracParams;

    simData.mrac.altitude.referenceState = nan(2, N);
    simData.mrac.altitude.error = nan(2, N);
    simData.mrac.altitude.KxHat = nan(2, N);
    simData.mrac.altitude.KrHat = nan(1, N);
    simData.mrac.altitude.KdHat = nan(1, N);
    simData.mrac.altitude.deltaThrust = nan(1, N);
    simData.mrac.altitude.thrustHover = nan(1, N);

    simData.mrac.attitude.referenceState = nan(2, 3, N);
    simData.mrac.attitude.error = nan(2, 3, N);
    simData.mrac.attitude.KxHat = nan(2, 3, N);
    simData.mrac.attitude.KrHat = nan(3, N);
    simData.mrac.attitude.KdHat = nan(3, N);
    simData.mrac.attitude.OHat = nan(3, N);

    positionState.integralError = zeros(3,1);
    altitudeState = [];
    attitudeState = [];

    firstRef = GetReferenceAtStep(flightPlan, 1);

    posOut = InitializePositionXYControllerOutput(firstRef.psi);
    altOut = InitializeMRACAltitudeOutput(quadConfig);
    attOut = InitializeMRACAttitudeOutput(posOut.attitudeDesired);

    mixerOut = QuadrotorMixer(altOut.thrust, attOut.torque, quadConfig);

    for k = 1:N-1

        resetNow = ShouldResetAtLapStart(simConfig, flightPlan, k);

        if resetNow
            simData.state.x(:, k) = simConfig.reset.state(:);

            positionState.integralError = zeros(3,1);
            altitudeState = [];
            attitudeState = [];

            refReset = GetReferenceAtStep(flightPlan, k);

            posOut = InitializePositionXYControllerOutput(refReset.psi);
            altOut = InitializeMRACAltitudeOutput(quadConfig);
            attOut = InitializeMRACAttitudeOutput(posOut.attitudeDesired);

            mixerOut = QuadrotorMixer(altOut.thrust, attOut.torque, quadConfig);
        end

        stateNow = simData.state.x(:, k);

        controlStateNow = stateNow;
        controlStateNow(idx.position) = ApplyNoise('r', stateNow(idx.position), noiseParams);
        controlStateNow(idx.velocity) = ApplyNoise('v', stateNow(idx.velocity), noiseParams);
        controlStateNow(idx.attitude) = ApplyNoise('eta', stateNow(idx.attitude), noiseParams);
        controlStateNow(idx.bodyRate) = ApplyNoise('omega', stateNow(idx.bodyRate), noiseParams);

        simData.state.xMeasured(:, k) = controlStateNow;

        ref = GetReferenceAtStep(flightPlan, k);
        condition = GetConditionAtStep(flightPlan, k);

        quadControl = quadConfig;

        quadPlant = quadConfig;
        quadPlant.mass = condition.mass;

        if isfield(quadConfig, "control") && isfield(quadConfig.control, "maxThrustFactor")
            quadPlant.control.maxThrust = ...
                quadConfig.control.maxThrustFactor*quadPlant.mass*quadConfig.gravity;
        end

        disturbance.forceInertial = condition.forceInertial;
        disturbance.forceBody = zeros(3,1);

        if isfield(condition, "torqueBody")
            disturbance.torqueBody = condition.torqueBody;
        else
            disturbance.torqueBody = zeros(3,1);
        end

        if k == 1 || mod(k-1, positionStep) == 0 || resetNow
            [posOut, positionState] = PositionControlLoopXY( ...
                controlStateNow, ...
                ref, ...
                controllersConfig.position, ...
                quadControl, ...
                positionState);
        end

        if k == 1 || mod(k-1, altitudeStep) == 0 || resetNow
            [altOut, altitudeState] = MRACAltitudeControlLoop( ...
                controlStateNow, ...
                ref.r(3), ...
                controllersConfig.altitude, ...
                mracParams.altitude, ...
                altitudeState, ...
                quadControl);
        end

        if k == 1 || mod(k-1, attitudeStep) == 0 || resetNow
            [attOut, attitudeState] = MRACAttitudeControlLoop( ...
                controlStateNow, ...
                posOut.attitudeDesired, ...
                controllersConfig.attitude, ...
                mracParams.attitude, ...
                attitudeState, ...
                quadControl);
        end

        mixerOut = QuadrotorMixer(altOut.thrust, attOut.torque, quadControl);

        simData.state.xDot(:, k) = QuadrotorDynamics( ...
            stateNow, ...
            mixerOut.motorOmega, ...
            quadPlant, ...
            disturbance);

        simData.state.x(:, k+1) = IntegrateStep( ...
            stateNow, ...
            mixerOut.motorOmega, ...
            quadPlant, ...
            disturbance, ...
            simConfig);
        simData.cmd.thrust(k) = altOut.thrust;
        simData.cmd.thrustRaw(k) = altOut.thrustRaw;
        simData.cmd.torque(:, k) = attOut.torque;
        simData.cmd.motorOmega(:, k) = mixerOut.motorOmega;
        simData.cmd.omegaSquared(:, k) = mixerOut.omegaSquared;
        simData.cmd.attitudeDesired(:, k) = posOut.attitudeDesired;
        simData.cmd.accelerationCommand(:, k) = posOut.accelerationCommand;

        simData.error.position(:, k) = ref.r - stateNow(idx.position);
        simData.error.velocity(:, k) = ref.v - stateNow(idx.velocity);
        simData.error.attitude(:, k) = WrapAngle(posOut.attitudeDesired - stateNow(idx.attitude));
        simData.error.angularVelocity(:, k) = -stateNow(idx.bodyRate);

        simData.diagnostic.motorSaturation(:, k) = mixerOut.saturated;
        simData.diagnostic.anyMotorSaturation(k) = mixerOut.anySaturated;
        simData.diagnostic.tiltSaturation(:, k) = posOut.tiltSaturated;
        simData.diagnostic.thrustSaturation(k) = altOut.thrustSaturated;
        simData.diagnostic.resetApplied(k) = resetNow;

        simData = StoreMRACData(simData, altOut, attOut, k);
    end

    k = N;

    resetNow = ShouldResetAtLapStart(simConfig, flightPlan, k);

    if resetNow
        simData.state.x(:, k) = simConfig.reset.state(:);

        positionState.integralError = zeros(3,1);
        altitudeState = [];
        attitudeState = [];

        refReset = GetReferenceAtStep(flightPlan, k);

        posOut = InitializePositionXYControllerOutput(refReset.psi);
        altOut = InitializeMRACAltitudeOutput(quadConfig);
        attOut = InitializeMRACAttitudeOutput(posOut.attitudeDesired);

        mixerOut = QuadrotorMixer(altOut.thrust, attOut.torque, quadConfig);
        simData.diagnostic.resetApplied(k) = true;
    end

    stateNow = simData.state.x(:, k);

    controlStateNow = stateNow;
    controlStateNow(idx.position) = ApplyNoise('r', stateNow(idx.position), noiseParams);
    controlStateNow(idx.velocity) = ApplyNoise('v', stateNow(idx.velocity), noiseParams);
    controlStateNow(idx.attitude) = ApplyNoise('eta', stateNow(idx.attitude), noiseParams);
    controlStateNow(idx.bodyRate) = ApplyNoise('omega', stateNow(idx.bodyRate), noiseParams);

    simData.state.xMeasured(:, k) = controlStateNow;

    ref = GetReferenceAtStep(flightPlan, k);
    condition = GetConditionAtStep(flightPlan, k);

    quadPlant = quadConfig;
    quadPlant.mass = condition.mass;

    disturbance.forceInertial = condition.forceInertial;
    disturbance.forceBody = zeros(3,1);

    if isfield(condition, "torqueBody")
        disturbance.torqueBody = condition.torqueBody;
    else
        disturbance.torqueBody = zeros(3,1);
    end

    simData.state.xDot(:, k) = QuadrotorDynamics( ...
        stateNow, ...
        mixerOut.motorOmega, ...
        quadPlant, ...
        disturbance);

    simData.cmd.thrust(k) = altOut.thrust;
    simData.cmd.thrustRaw(k) = altOut.thrustRaw;
    simData.cmd.torque(:, k) = attOut.torque;
    simData.cmd.motorOmega(:, k) = mixerOut.motorOmega;
    simData.cmd.omegaSquared(:, k) = mixerOut.omegaSquared;
    simData.cmd.attitudeDesired(:, k) = posOut.attitudeDesired;
    simData.cmd.accelerationCommand(:, k) = posOut.accelerationCommand;

    simData.error.position(:, k) = ref.r - stateNow(idx.position);
    simData.error.velocity(:, k) = ref.v - stateNow(idx.velocity);
    simData.error.attitude(:, k) = WrapAngle(posOut.attitudeDesired - stateNow(idx.attitude));
    simData.error.angularVelocity(:, k) = -stateNow(idx.bodyRate);

    simData = StoreMRACData(simData, altOut, attOut, k);

    simData.summary.positionControllerActualFrequency = ...
        controllersConfig.position.actualFrequency;

    simData.summary.altitudeControllerActualFrequency = ...
        controllersConfig.altitude.actualFrequency;

    simData.summary.attitudeControllerActualFrequency = ...
        controllersConfig.attitude.actualFrequency;

    simData.summary.finalPositionError = simData.error.position(:, end);
    simData.summary.maxPositionErrorNorm = ...
        max(vecnorm(simData.error.position, 2, 1));
end

function resetNow = ShouldResetAtLapStart(simConfig, flightPlan, k)
    resetNow = false;

    if ~isfield(simConfig, "reset")
        return;
    end

    if ~isfield(simConfig.reset, "stateEachLap")
        return;
    end

    if ~simConfig.reset.stateEachLap
        return;
    end

    if k <= 1
        return;
    end

    lapNow = flightPlan.index.lap(k);
    lapPrevious = flightPlan.index.lap(k-1);

    resetNow = lapNow ~= lapPrevious;
end

function posOut = InitializePositionXYControllerOutput(psiDesired)
    posOut = struct();
    posOut.attitudeDesired = [0; 0; psiDesired];
    posOut.accelerationCommand = zeros(3,1);
    posOut.accelerationFeedback = zeros(3,1);
    posOut.positionError = zeros(3,1);
    posOut.velocityError = zeros(3,1);
    posOut.integralError = zeros(3,1);
    posOut.tiltSaturated = false(2,1);
end

function altOut = InitializeMRACAltitudeOutput(quadConfig)
    altOut = struct();
    altOut.thrust = quadConfig.mass*quadConfig.gravity;
    altOut.thrustRaw = altOut.thrust;
    altOut.deltaThrust = 0;
    altOut.thrustHover = altOut.thrust;
    altOut.referenceState = zeros(2,1);
    altOut.error = zeros(2,1);
    altOut.KxHat = zeros(2,1);
    altOut.KrHat = 0;
    altOut.KdHat = 0;
    altOut.thrustSaturated = false;
end

function attOut = InitializeMRACAttitudeOutput(attitudeDesired)
    attOut = struct();
    attOut.torque = zeros(3,1);
    attOut.attitudeDesired = attitudeDesired;
    attOut.attitudeError = zeros(3,1);
    attOut.rateError = zeros(3,1);
    attOut.referenceState = zeros(2,3);
    attOut.error = zeros(2,3);
    attOut.KxHat = zeros(2,3);
    attOut.KrHat = zeros(3,1);
    attOut.KdHat = zeros(3,1);
    attOut.OHat = zeros(3,1);
end

function simData = StoreMRACData(simData, altOut, attOut, k)
    simData.mrac.altitude.referenceState(:, k) = altOut.referenceState;
    simData.mrac.altitude.error(:, k) = altOut.error;
    simData.mrac.altitude.KxHat(:, k) = altOut.KxHat;
    simData.mrac.altitude.KrHat(k) = altOut.KrHat;
    simData.mrac.altitude.KdHat(k) = altOut.KdHat;
    simData.mrac.altitude.deltaThrust(k) = altOut.deltaThrust;
    simData.mrac.altitude.thrustHover(k) = altOut.thrustHover;

    simData.mrac.attitude.referenceState(:, :, k) = attOut.referenceState;
    simData.mrac.attitude.error(:, :, k) = attOut.error;
    simData.mrac.attitude.KxHat(:, :, k) = attOut.KxHat;
    simData.mrac.attitude.KrHat(:, k) = attOut.KrHat;
    simData.mrac.attitude.KdHat(:, k) = attOut.KdHat;
    simData.mrac.attitude.OHat(:, k) = attOut.OHat;
end
