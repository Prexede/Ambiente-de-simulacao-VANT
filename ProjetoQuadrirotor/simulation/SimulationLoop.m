function simData = SimulationLoop(simConfig, quadConfig, controllersConfig, flightPlan)
% SimulationLoop
% -------------------------------------------------------------------------
% Loop unico de simulacao. Esta funcao nao gera trajetoria, nao estima
% modelo, nao estima ganhos e nao faz plots.
% -------------------------------------------------------------------------

    ValidateStructFields(simConfig.time, ["t", "Ts", "tf", "segmentTime", "lapTime"], "simConfig.time");
    ValidateStructFields(quadConfig, ["mass", "inertia", "armLength", "gravity", "kThrust", "kDrag"], "quadConfig");

    idx = StateIndex();

    t = simConfig.time.t(:).';
    Ts = simConfig.time.Ts;
    N = numel(t);

    positionStep = max(1, round(1/(controllersConfig.position.updateFrequency*Ts)));
    attitudeStep = max(1, round(1/(controllersConfig.attitude.updateFrequency*Ts)));

    controllersConfig.position.updateStep = positionStep;
    controllersConfig.position.actualFrequency = 1/(positionStep*Ts);
    controllersConfig.position.updatePeriod = positionStep*Ts;
    controllersConfig.position.integralLimit = inf(3,1);

    controllersConfig.attitude.updateStep = attitudeStep;
    controllersConfig.attitude.actualFrequency = 1/(attitudeStep*Ts);
    controllersConfig.attitude.updatePeriod = attitudeStep*Ts;
    controllersConfig.attitude.integralLimit = inf(3,1);

    simData = InitializeSimulationData(simConfig, quadConfig, controllersConfig, flightPlan);

    positionState.integralError = zeros(3,1);
    attitudeState.integralError = zeros(3,1);

    firstRef = GetReferenceAtStep(flightPlan, 1);

    posOut.thrust = quadConfig.mass*quadConfig.gravity;
    posOut.thrustRaw = posOut.thrust;
    posOut.attitudeDesired = [0; 0; firstRef.psi];
    posOut.accelerationCommand = zeros(3,1);
    posOut.accelerationFeedback = zeros(3,1);
    posOut.positionError = zeros(3,1);
    posOut.velocityError = zeros(3,1);
    posOut.integralError = zeros(3,1);
    posOut.tiltSaturated = false(2,1);
    posOut.thrustSaturated = false;

    attOut.torque = zeros(3,1);
    attOut.attitudeError = zeros(3,1);
    attOut.rateError = zeros(3,1);
    attOut.integralError = zeros(3,1);
    attOut.attitudeDesired = posOut.attitudeDesired;

    mixerOut = QuadrotorMixer(posOut.thrust, attOut.torque, quadConfig);

    for k = 1:N-1
        stateNow = simData.state.x(:, k);
        ref = GetReferenceAtStep(flightPlan, k);
        condition = GetConditionAtStep(flightPlan, k);

        quadNow = quadConfig;
        quadNow.mass = condition.mass;
        quadNow.control.maxThrust = quadConfig.control.maxThrustFactor*condition.mass*quadConfig.gravity;

        disturbance.forceInertial = condition.forceInertial;
        disturbance.forceBody = zeros(3,1);

        if k == 1 || mod(k-1, positionStep) == 0
            [posOut, positionState] = PositionControlLoop( ...
                stateNow, ...
                ref, ...
                controllersConfig.position, ...
                quadNow, ...
                positionState);
        end

        if k == 1 || mod(k-1, attitudeStep) == 0
            [attOut, attitudeState] = AttitudeControlLoop( ...
                stateNow, ...
                posOut.attitudeDesired, ...
                controllersConfig.attitude, ...
                attitudeState);
        end

        mixerOut = QuadrotorMixer(posOut.thrust, attOut.torque, quadNow);

        simData.state.xDot(:, k) = QuadrotorDynamics(stateNow, mixerOut.motorOmega, quadNow, disturbance);
        simData.state.x(:, k+1) = IntegrateStep(stateNow, mixerOut.motorOmega, quadNow, disturbance, simConfig);

        simData.cmd.thrust(k) = posOut.thrust;
        simData.cmd.thrustRaw(k) = posOut.thrustRaw;
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
        simData.diagnostic.thrustSaturation(k) = posOut.thrustSaturated;
    end

    % Preenche ultima amostra para manter vetores completos.
    k = N;
    stateNow = simData.state.x(:, k);
    ref = GetReferenceAtStep(flightPlan, k);
    condition = GetConditionAtStep(flightPlan, k);

    quadNow = quadConfig;
    quadNow.mass = condition.mass;
    quadNow.control.maxThrust = quadConfig.control.maxThrustFactor*condition.mass*quadConfig.gravity;

    disturbance.forceInertial = condition.forceInertial;
    disturbance.forceBody = zeros(3,1);

    simData.state.xDot(:, k) = QuadrotorDynamics(stateNow, mixerOut.motorOmega, quadNow, disturbance);

    simData.cmd.thrust(k) = posOut.thrust;
    simData.cmd.thrustRaw(k) = posOut.thrustRaw;
    simData.cmd.torque(:, k) = attOut.torque;
    simData.cmd.motorOmega(:, k) = mixerOut.motorOmega;
    simData.cmd.omegaSquared(:, k) = mixerOut.omegaSquared;
    simData.cmd.attitudeDesired(:, k) = posOut.attitudeDesired;
    simData.cmd.accelerationCommand(:, k) = posOut.accelerationCommand;

    simData.error.position(:, k) = ref.r - stateNow(idx.position);
    simData.error.velocity(:, k) = ref.v - stateNow(idx.velocity);
    simData.error.attitude(:, k) = WrapAngle(posOut.attitudeDesired - stateNow(idx.attitude));
    simData.error.angularVelocity(:, k) = -stateNow(idx.bodyRate);

    simData.summary.positionControllerActualFrequency = controllersConfig.position.actualFrequency;
    simData.summary.attitudeControllerActualFrequency = controllersConfig.attitude.actualFrequency;
    simData.summary.finalPositionError = simData.error.position(:, end);
    simData.summary.maxPositionErrorNorm = max(vecnorm(simData.error.position, 2, 1));
end