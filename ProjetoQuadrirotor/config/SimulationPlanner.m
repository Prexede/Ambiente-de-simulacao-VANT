function simConfig = SimulationPlanner(varargin)
% SimulationPlanner
% -------------------------------------------------------------------------
% Define as configuracoes temporais da simulacao e os metadados da
% trajetoria.
%
% Conceitos usados:
%   waypoint          -> ponto da trajetoria;
%   segmento          -> movimento do waypoint i para o waypoint i+1;
%   volta             -> execucao completa de todos os segmentos uma vez;
%   segmentTime       -> tempo de cada movimento entre dois waypoints;
%   lapTime           -> tempo para completar uma volta;
%   repetitions       -> numero de voltas completas.
% -------------------------------------------------------------------------

    p = inputParser;
    addParameter(p, "TrajectoryType", "quad");
    addParameter(p, "Ts", 0.01, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, "SegmentTime", 5, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, "Repetitions", 1, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, "YawDesired", 0, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, "IntegrationMethod", "RK4");
    addParameter(p, "InitialState", zeros(12,1), @(x) isnumeric(x) && numel(x) == 12);
    parse(p, varargin{:});

    trajectoryType = string(p.Results.TrajectoryType);
    Ts = p.Results.Ts;
    segmentTime = p.Results.SegmentTime;
    repetitions = round(p.Results.Repetitions);
    yawDesired = p.Results.YawDesired;
    integrationMethod = string(p.Results.IntegrationMethod);
    initialState = p.Results.InitialState(:);

    waypoints = BuildWaypoints(trajectoryType);
    numWaypointsPerLap = size(waypoints, 1);
    numSegmentsPerLap = numWaypointsPerLap - 1;

    if numSegmentsPerLap < 1
        error('A trajetoria precisa ter pelo menos dois waypoints.');
    end

    lapTime = numSegmentsPerLap * segmentTime;
    totalSegments = numSegmentsPerLap * repetitions;
    tf = totalSegments * segmentTime;

    t = 0:Ts:tf;
    if abs(t(end) - tf) > 10*eps(max(1, tf))
        t = [t, tf];
    end

    simConfig = struct();

    simConfig.time.Ts = Ts;
    simConfig.time.t = t(:).';
    simConfig.time.tf = tf;
    simConfig.time.segmentTime = segmentTime;
    simConfig.time.lapTime = lapTime;
    simConfig.time.repetitions = repetitions;
    simConfig.time.totalSegments = totalSegments;

    simConfig.trajectory.type = trajectoryType;
    simConfig.trajectory.yawDesired = yawDesired;
    simConfig.trajectory.waypoints = waypoints;
    simConfig.trajectory.numWaypointsPerLap = numWaypointsPerLap;
    simConfig.trajectory.numSegmentsPerLap = numSegmentsPerLap;

    simConfig.integration.method = integrationMethod;
    simConfig.initialState = initialState;
end
