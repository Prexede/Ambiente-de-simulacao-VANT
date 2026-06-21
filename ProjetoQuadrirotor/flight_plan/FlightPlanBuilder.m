function flightPlan = FlightPlanBuilder(simConfig, quadConfig, varargin)
% FlightPlanBuilder
% -------------------------------------------------------------------------
% Gera uma estrutura unica contendo:
%   - referencia de posicao, velocidade, aceleracao e yaw;
%   - indice da volta e do segmento em cada amostra;
%   - disturbios avaliados ponto a ponto, definidos por volta.
%
% Disturbios permitidos:
%   "nominal"    -> sem disturbios
%   "mass"       -> variacao de massa por volta
%   "wind"       -> forca externa por volta
%   "masswind"   -> variacao de massa + forca externa por volta
%
% A massa sempre e informada como delta em relacao a massa nominal.
% O vento sempre e informado como forca em Newton no referencial inercial.
% -------------------------------------------------------------------------

    p = inputParser;
    addParameter(p, "DisturbanceMode", "nominal");
    addParameter(p, "MassByLap", []);
    addParameter(p, "WindByLap", []);
    parse(p, varargin{:});

    trajectory = GenerateTrajectory(simConfig);

    disturbanceConfig = struct();
    disturbanceConfig.mode = lower(string(p.Results.DisturbanceMode));
    disturbanceConfig.massByLap = p.Results.MassByLap;
    disturbanceConfig.windByLap = p.Results.WindByLap;

    condition = GenerateDisturbanceProfile( ...
        simConfig, ...
        quadConfig, ...
        disturbanceConfig, ...
        trajectory.index.lap);

    flightPlan = struct();
    flightPlan.t = simConfig.time.t;
    flightPlan.ref = trajectory.ref;
    flightPlan.index = trajectory.index;
    flightPlan.waypoints = trajectory.waypoints;
    flightPlan.condition = condition;
    flightPlan.disturbance = disturbanceConfig;

    flightPlan.info.trajectoryType = simConfig.trajectory.type;
    flightPlan.info.segmentTime = simConfig.time.segmentTime;
    flightPlan.info.lapTime = simConfig.time.lapTime;
    flightPlan.info.repetitions = simConfig.time.repetitions;
    flightPlan.info.numSegmentsPerLap = simConfig.trajectory.numSegmentsPerLap;
end