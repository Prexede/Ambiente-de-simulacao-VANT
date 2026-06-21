function simData = SimulationLoop(simConfig, quadConfig, controllersConfig, flightPlan)
% SimulationLoop
% -------------------------------------------------------------------------
% Loop unico de simulacao.
%
% Esta funcao:
%   - nao gera trajetoria;
%   - nao estima modelo;
%   - nao estima ganhos;
%   - nao faz plots.
%
% Importante:
%   quadControl -> modelo nominal usado pelo controlador.
%   quadPlant   -> modelo real/perturbado usado pela dinamica.
%
% Assim, mudancas de massa entram como disturbio real, pois o controlador
% nao recebe a massa alterada.
% -------------------------------------------------------------------------

    ValidateStructFields(simConfig.time, ...
        ["t", "Ts", "tf", "segmentTime", "lapTime"], ...
        "simConfig.time");

    ValidateStructFields(quadConfig, ...
        ["mass", "inertia", "armLength", "gravity", "kThrust", "kDrag"], ...
        "quadConfig");

    idx = StateIndex();

    t = simConfig.time.t(:).';
    Ts = simConfig.time.Ts;
    N = numel(t);

    %% Frequencias dos controladores
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

    %% Inicializacao da saida
    simData = InitializeSimulationData( ...
        simConfig, ...
        quadConfig, ...
        controllersConfig, ...
        flightPlan);

    %% Estados internos dos controladores
    positionState.integralError = zeros(3,1);
    attitudeState.integralError = zeros(3,1);

    %% Saidas iniciais
    firstRef = GetReferenceAtStep(flightPlan, 1);

    posOut = InitializePositionControllerOutput(quadConfig, firstRef.psi);
    attOut = InitializeAttitudeControllerOutput(posOut.attitudeDesired);

    mixerOut = QuadrotorMixer(posOut.thrust, attOut.torque, quadConfig);

    %% Loop principal
    for k = 1:N-1

        resetNow = ShouldResetAtLapStart(simConfig, flightPlan, k);

        if resetNow
            simData.state.x(:, k) = simConfig.reset.state(:);

            positionState.integralError = zeros(3,1);
            attitudeState.integralError = zeros(3,1);

            refReset = GetReferenceAtStep(flightPlan, k);

            posOut = InitializePositionControllerOutput(quadConfig, refReset.psi);
            attOut = InitializeAttitudeControllerOutput(posOut.attitudeDesired);

            mixerOut = QuadrotorMixer(posOut.thrust, attOut.torque, quadConfig);
        end

        stateNow = simData.state.x(:, k);

        ref = GetReferenceAtStep(flightPlan, k);
        condition = GetConditionAtStep(flightPlan, k);

        %% Modelo usado pelo controlador: nominal
        quadControl = quadConfig;

        %% Modelo usado pela planta: perturbado
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

        %% Controle de posicao
        % O controlador recebe quadControl, ou seja, o modelo nominal.
        if k == 1 || mod(k-1, positionStep) == 0 || resetNow
            [posOut, positionState] = PositionControlLoop( ...
                stateNow, ...
                ref, ...
                controllersConfig.position, ...
                quadControl, ...
                positionState);
        end

        %% Controle de atitude
        if k == 1 || mod(k-1, attitudeStep) == 0 || resetNow
            [attOut, attitudeState] = AttitudeControlLoop( ...
                stateNow, ...
                posOut.attitudeDesired, ...
                controllersConfig.attitude, ...
                attitudeState);
        end

        %% Mixer
        % O mixer usa os parametros fisicos nominais dos atuadores.
        mixerOut = QuadrotorMixer(posOut.thrust, attOut.torque, quadControl);

        %% Dinamica e integracao
        % A dinamica recebe quadPlant, ou seja, a massa perturbada.
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

        %% Armazenamento dos comandos
        simData.cmd.thrust(k) = posOut.thrust;
        simData.cmd.thrustRaw(k) = posOut.thrustRaw;
        simData.cmd.torque(:, k) = attOut.torque;
        simData.cmd.motorOmega(:, k) = mixerOut.motorOmega;
        simData.cmd.omegaSquared(:, k) = mixerOut.omegaSquared;
        simData.cmd.attitudeDesired(:, k) = posOut.attitudeDesired;
        simData.cmd.accelerationCommand(:, k) = posOut.accelerationCommand;

        %% Erros
        simData.error.position(:, k) = ref.r - stateNow(idx.position);
        simData.error.velocity(:, k) = ref.v - stateNow(idx.velocity);
        simData.error.attitude(:, k) = WrapAngle(posOut.attitudeDesired - stateNow(idx.attitude));
        simData.error.angularVelocity(:, k) = -stateNow(idx.bodyRate);

        %% Diagnosticos
        simData.diagnostic.motorSaturation(:, k) = mixerOut.saturated;
        simData.diagnostic.anyMotorSaturation(k) = mixerOut.anySaturated;
        simData.diagnostic.tiltSaturation(:, k) = posOut.tiltSaturated;
        simData.diagnostic.thrustSaturation(k) = posOut.thrustSaturated;
        simData.diagnostic.resetApplied(k) = resetNow;
    end

    %% Ultima amostra
    k = N;

    resetNow = ShouldResetAtLapStart(simConfig, flightPlan, k);

    if resetNow
        simData.state.x(:, k) = simConfig.reset.state(:);

        positionState.integralError = zeros(3,1);
        attitudeState.integralError = zeros(3,1);

        refReset = GetReferenceAtStep(flightPlan, k);

        posOut = InitializePositionControllerOutput(quadConfig, refReset.psi);
        attOut = InitializeAttitudeControllerOutput(posOut.attitudeDesired);

        mixerOut = QuadrotorMixer(posOut.thrust, attOut.torque, quadConfig);

        simData.diagnostic.resetApplied(k) = true;
    end

    stateNow = simData.state.x(:, k);

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

    %% Resumo
    simData.summary.positionControllerActualFrequency = ...
        controllersConfig.position.actualFrequency;

    simData.summary.attitudeControllerActualFrequency = ...
        controllersConfig.attitude.actualFrequency;

    simData.summary.finalPositionError = simData.error.position(:, end);
    simData.summary.maxPositionErrorNorm = ...
        max(vecnorm(simData.error.position, 2, 1));
end

% =========================================================================
% Funcoes auxiliares locais
% =========================================================================

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

function posOut = InitializePositionControllerOutput(quadConfig, psiDesired)
    posOut = struct();

    posOut.thrust = quadConfig.mass*quadConfig.gravity;
    posOut.thrustRaw = posOut.thrust;
    posOut.attitudeDesired = [0; 0; psiDesired];

    posOut.accelerationCommand = zeros(3,1);
    posOut.accelerationFeedback = zeros(3,1);

    posOut.positionError = zeros(3,1);
    posOut.velocityError = zeros(3,1);
    posOut.integralError = zeros(3,1);

    posOut.tiltSaturated = false(2,1);
    posOut.thrustSaturated = false;
end

function attOut = InitializeAttitudeControllerOutput(attitudeDesired)
    attOut = struct();

    attOut.torque = zeros(3,1);

    attOut.attitudeError = zeros(3,1);
    attOut.rateError = zeros(3,1);
    attOut.integralError = zeros(3,1);
    attOut.attitudeDesired = attitudeDesired;
end