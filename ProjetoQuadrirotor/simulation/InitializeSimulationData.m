function simData = InitializeSimulationData(simConfig, quadConfig, controllersConfig, flightPlan)
% InitializeSimulationData
% -------------------------------------------------------------------------
% Pre-aloca a struct de saida da simulacao. Esta funcao nao calcula
% dinamica, controle nem plots.
% -------------------------------------------------------------------------

    t = simConfig.time.t(:).';
    N = numel(t);

    simData = struct();
    simData.t = t;

    simData.state.x = nan(12, N);
    simData.state.xDot = nan(12, N);
    simData.state.x(:, 1) = simConfig.initialState(:);

    simData.ref.r = flightPlan.ref.r;
    simData.ref.v = flightPlan.ref.v;
    simData.ref.a = flightPlan.ref.a;
    simData.ref.psi = flightPlan.ref.psi;

    simData.cmd.thrust = nan(1, N);
    simData.cmd.thrustRaw = nan(1, N);
    simData.cmd.torque = nan(3, N);
    simData.cmd.motorOmega = nan(4, N);
    simData.cmd.omegaSquared = nan(4, N);
    simData.cmd.attitudeDesired = nan(3, N);
    simData.cmd.accelerationCommand = nan(3, N);

    simData.error.position = nan(3, N);
    simData.error.velocity = nan(3, N);
    simData.error.attitude = nan(3, N);
    simData.error.angularVelocity = nan(3, N);

    simData.condition.mass = flightPlan.condition.mass;
    simData.condition.windInertial = flightPlan.condition.windInertial;
    simData.condition.forceInertial = flightPlan.condition.forceInertial;

    if isfield(flightPlan.condition, "torqueBody")
        simData.condition.torqueBody = flightPlan.condition.torqueBody;
    else
        simData.condition.torqueBody = zeros(3, N);
    end

    simData.condition.lap = flightPlan.index.lap;
    simData.condition.segment = flightPlan.index.segment;
    simData.condition.segmentInLap = flightPlan.index.segmentInLap;

    simData.diagnostic.motorSaturation = false(4, N);
    simData.diagnostic.anyMotorSaturation = false(1, N);
    simData.diagnostic.tiltSaturation = false(2, N);
    simData.diagnostic.thrustSaturation = false(1, N);

    simData.diagnostic.resetApplied = false(1, N);

    simData.config.sim = simConfig;
    simData.config.quad = quadConfig;
    simData.config.controllers = controllersConfig;
    simData.config.flightPlan = flightPlan;
end