clear;
close all;
clc;

projectRoot = fileparts(mfilename("fullpath"));
addpath(genpath(projectRoot));

trajectoryType = "quad";
Ts = 0.01;
segmentTime = 20;
repetitions = 3;
yawDesired = 0;
integrationMethod = "euler";

positionFrequency = 10;
altitudeFrequency = 10;
attitudeFrequency = 100;

saveFigures = false;
saveResults = false;
outputFolder = "comparacao_controladores_quad";

controllerColors = [
    1.0000    0.0000    0.0000;
    0.0000    0.0000    1.0000;
    0.0000    0.6000    0.0000
];

referenceColor = [0.0000 0.0000 0.0000];

quadConfig = QuadrotorModel( ...
    "Material", "CarbonFiber", ...
    "Geometry", [20 5 1], ...
    "HoverSpeed", 1000, ...
    "OmegaMin", 0, ...
    "OmegaMax", 5000, ...
    "MaxTiltAngle", deg2rad(30), ...
    "MaxThrustFactor", 5);

simConfig = SimulationPlanner( ...
    "TrajectoryType", "quad", ...
    "Ts", 0.01, ...
    "SegmentTime", 20, ...
    "Repetitions", 3, ...
    "YawDesired", 0, ...
    "YawMode", "const", ...
    "YawTargetXY", [0 0], ...
    "IntegrationMethod", "euler", ...
    "InitialState", zeros(12,1), ...
    "ResetStateEachLap", false);

flightPlan = FlightPlanBuilder( ...
    simConfig, ...
    quadConfig, ...
    "DisturbanceMode", "mass", ...
    "MassByLap", [ ...
        1,  0.00; ...
        2, +2.00; ...
        3, +2.00; ...
        4, +2; ...
        5, +4], ...
    "WindByLap", [ ...
        1, 0.0, 0.0, 0.0; ...
        2, 0.0, 0.0, 0.0; ...
        3, 0.0, 0.0, 0.5]);

noiseParams = NoiseParams();
noiseParams.enable = false;
noiseParams.disturbanceType = "whiteNoise";
noiseParams.r.enable = false;
noiseParams.v.enable = false;
noiseParams.eta.enable = false;
noiseParams.omega.enable = false;

mracParams = MRACParams(quadConfig);

controllerCases = struct( ...
    "name", {"PD", "PID", "MRAC"}, ...
    "positionType", {"PD", "PD", "PD"}, ...
    "altitudeType", {"PD", "PID", "MRAC"}, ...
    "attitudeType", {"PD", "PID", "MRAC"});

numCases = numel(controllerCases);
results = struct([]);

fprintf("\n--- Comparacao de controladores ---\n");
fprintf("Trajetoria: %s\n", trajectoryType);
fprintf("Tempo por segmento: %.2f s\n", segmentTime);
fprintf("Repeticoes: %d\n", repetitions);
fprintf("Tempo final: %.2f s\n", simConfig.time.tf);

for i = 1:numCases
    fprintf("\nRodando caso: %s\n", controllerCases(i).name);

    controllersConfig = ControllersConfig( ...
        "SimulationModel", "MRAC_Test", ...
        "PositionType", controllerCases(i).positionType, ...
        "AltitudeType", controllerCases(i).altitudeType, ...
        "AttitudeType", controllerCases(i).attitudeType, ...
        "PositionFrequency", positionFrequency, ...
        "AltitudeFrequency", altitudeFrequency, ...
        "AttitudeFrequency", attitudeFrequency);

    simData = SimulationLoop_MRAC_Test( ...
        simConfig, ...
        quadConfig, ...
        controllersConfig, ...
        mracParams, ...
        flightPlan, ...
        noiseParams);

    results(i).name = string(controllerCases(i).name);
    results(i).color = controllerColors(i,:);
    results(i).controllersConfig = controllersConfig;
    results(i).simData = simData;
end

summaryTable = BuildSummaryTable(results);

fprintf("\n--- Resumo numerico ---\n");
disp(summaryTable);

PlotTrajectoryComparison(results, referenceColor);
PlotPositionErrorComparison(results, repetitions);
PlotAltitudeComparison(results, referenceColor, repetitions);
PlotAttitudeErrorComparison(results, repetitions);
PlotThrustComparison(results, repetitions);

if saveResults
    if ~exist(outputFolder, "dir")
        mkdir(outputFolder);
    end

    save(fullfile(outputFolder, "comparacao_controladores_quad.mat"), ...
        "results", ...
        "summaryTable", ...
        "simConfig", ...
        "quadConfig", ...
        "flightPlan", ...
        "noiseParams", ...
        "controllerColors", ...
        "referenceColor");
end

if saveFigures
    if ~exist(outputFolder, "dir")
        mkdir(outputFolder);
    end

    figHandles = findall(0, "Type", "figure");

    for i = 1:numel(figHandles)
        figName = figHandles(i).Name;
        figName = matlab.lang.makeValidName(figName);
        exportgraphics(figHandles(i), ...
            fullfile(outputFolder, figName + ".png"), ...
            "Resolution", 300, ...
            "BackgroundColor", "white");
    end
end

function summaryTable = BuildSummaryTable(results)

    numCases = numel(results);

    Controller = strings(numCases, 1);
    RMSE_Position_m = zeros(numCases, 1);
    Max_Position_Error_m = zeros(numCases, 1);
    RMSE_Attitude_deg = zeros(numCases, 1);
    Max_Attitude_Error_deg = zeros(numCases, 1);

    for i = 1:numCases
        simData = results(i).simData;

        ePos = vecnorm(simData.error.position, 2, 1);
        eAtt = rad2deg(vecnorm(simData.error.attitude, 2, 1));

        ePos = CleanVector(ePos);
        eAtt = CleanVector(eAtt);

        Controller(i) = results(i).name;
        RMSE_Position_m(i) = RMSValue(ePos);
        Max_Position_Error_m(i) = max(ePos);
        RMSE_Attitude_deg(i) = RMSValue(eAtt);
        Max_Attitude_Error_deg(i) = max(eAtt);
    end

    summaryTable = table( ...
        Controller, ...
        RMSE_Position_m, ...
        Max_Position_Error_m, ...
        RMSE_Attitude_deg, ...
        Max_Attitude_Error_deg);
end

function PlotTrajectoryComparison(results, referenceColor)

    idx = StateIndex();

    fig = figure("Name", "Comparacao_Trajetoria_3D", ...
        "NumberTitle", "off", ...
        "Color", "w", ...
        "InvertHardcopy", "off");

    ax = axes("Parent", fig);
    hold(ax, "on");

    ref = results(1).simData.ref.r;

    plot3(ax, ref(1,:), ref(2,:), ref(3,:), ...
        "--", ...
        "Color", referenceColor, ...
        "LineWidth", 2.0, ...
        "DisplayName", "Referencia");

    for i = 1:numel(results)
        simData = results(i).simData;
        r = simData.state.x(idx.position, :);

        plot3(ax, r(1,:), r(2,:), r(3,:), ...
            "Color", results(i).color, ...
            "LineWidth", 1.5, ...
            "DisplayName", results(i).name);
    end

    ConfigureAxes(ax, true);

    xlabel(ax, "x [m]", "Color", "k");
    ylabel(ax, "y [m]", "Color", "k");
    zlabel(ax, "z [m]", "Color", "k");
    title(ax, "Comparacao da trajetoria 3D", "Color", "k");

    axis(ax, "equal");
    view(ax, 3);

    lgd = legend(ax, "show", "Location", "best");
    ConfigureLegend(lgd);
end

function PlotPositionErrorComparison(results, repetitions)

    fig = figure("Name", "Comparacao_Erro_Posicao", ...
        "NumberTitle", "off", ...
        "Color", "w", ...
        "InvertHardcopy", "off");

    ax = axes("Parent", fig);
    hold(ax, "on");

    for i = 1:numel(results)
        simData = results(i).simData;
        t = simData.t;
        ePos = vecnorm(simData.error.position, 2, 1);

        plot(ax, t, ePos, ...
            "Color", results(i).color, ...
            "LineWidth", 1.5, ...
            "DisplayName", results(i).name);
    end

    AddLapChangeLines(ax, results(1).simData, repetitions);

    ConfigureAxes(ax, false);

    xlabel(ax, "Tempo [s]", "Color", "k");
    ylabel(ax, "||e_r|| [m]", "Color", "k");
    title(ax, "Erro de posicao", "Color", "k");

    lgd = legend(ax, "show", "Location", "best");
    ConfigureLegend(lgd);
end

function PlotAltitudeComparison(results, referenceColor, repetitions)

    idx = StateIndex();

    fig = figure("Name", "Comparacao_Altitude", ...
        "NumberTitle", "off", ...
        "Color", "w", ...
        "InvertHardcopy", "off");

    ax = axes("Parent", fig);
    hold(ax, "on");

    t = results(1).simData.t;
    zRef = results(1).simData.ref.r(3,:);

    plot(ax, t, zRef, ...
        "--", ...
        "Color", referenceColor, ...
        "LineWidth", 2.0, ...
        "DisplayName", "z_d");

    for i = 1:numel(results)
        simData = results(i).simData;
        z = simData.state.x(idx.z, :);

        plot(ax, t, z, ...
            "Color", results(i).color, ...
            "LineWidth", 1.5, ...
            "DisplayName", results(i).name);
    end

    AddLapChangeLines(ax, results(1).simData, repetitions);

    ConfigureAxes(ax, false);

    xlabel(ax, "Tempo [s]", "Color", "k");
    ylabel(ax, "z [m]", "Color", "k");
    title(ax, "Rastreamento de altitude", "Color", "k");

    lgd = legend(ax, "show", "Location", "best");
    ConfigureLegend(lgd);
end

function PlotAttitudeErrorComparison(results, repetitions)

    fig = figure("Name", "Comparacao_Erro_Atitude", ...
        "NumberTitle", "off", ...
        "Color", "w", ...
        "InvertHardcopy", "off");

    ax = axes("Parent", fig);
    hold(ax, "on");

    for i = 1:numel(results)
        simData = results(i).simData;
        t = simData.t;
        eAtt = rad2deg(vecnorm(simData.error.attitude, 2, 1));

        plot(ax, t, eAtt, ...
            "Color", results(i).color, ...
            "LineWidth", 1.2, ...
            "DisplayName", results(i).name);
    end

    AddLapChangeLines(ax, results(1).simData, repetitions);

    ConfigureAxes(ax, false);

    xlabel(ax, "Tempo [s]", "Color", "k");
    ylabel(ax, "||e_\eta|| [graus]", "Color", "k");
    title(ax, "Erro de atitude", "Color", "k");

    lgd = legend(ax, "show", "Location", "best");
    ConfigureLegend(lgd);
end

function PlotThrustComparison(results, repetitions)

    fig = figure("Name", "Comparacao_Empuxo", ...
        "NumberTitle", "off", ...
        "Color", "w", ...
        "InvertHardcopy", "off");

    ax = axes("Parent", fig);
    hold(ax, "on");

    for i = 1:numel(results)
        simData = results(i).simData;
        t = simData.t;
        thrust = simData.cmd.thrust;

        plot(ax, t, thrust, ...
            "Color", results(i).color, ...
            "LineWidth", 1.5, ...
            "DisplayName", results(i).name);
    end

    AddLapChangeLines(ax, results(1).simData, repetitions);

    ConfigureAxes(ax, false);

    xlabel(ax, "Tempo [s]", "Color", "k");
    ylabel(ax, "Empuxo [N]", "Color", "k");
    title(ax, "Comando de empuxo", "Color", "k");

    lgd = legend(ax, "show", "Location", "best");
    ConfigureLegend(lgd);
end

function AddLapChangeLines(ax, simData, repetitions)

    t = simData.t(:).';

    if isfield(simData, "condition") && ...
       isfield(simData.condition, "lap")

        lap = simData.condition.lap(:).';
        changeIndex = find(diff(lap) ~= 0) + 1;
        changeTimes = t(changeIndex);

    else
        tStart = t(1);
        tEnd = t(end);
        lapDuration = (tEnd - tStart)/repetitions;
        changeTimes = tStart + lapDuration*(1:(repetitions - 1));
    end

    for k = 1:numel(changeTimes)
        xline(ax, changeTimes(k), "--", ...
            "Color", [0.25 0.25 0.25], ...
            "LineWidth", 1.0, ...
            "HandleVisibility", "off");
    end
end

function ConfigureAxes(ax, is3D)

    set(ax, ...
        "Color", "w", ...
        "XColor", "k", ...
        "YColor", "k", ...
        "GridColor", [0.65 0.65 0.65], ...
        "MinorGridColor", [0.80 0.80 0.80], ...
        "GridAlpha", 0.45, ...
        "MinorGridAlpha", 0.25);

    if is3D
        set(ax, "ZColor", "k");
    end

    grid(ax, "on");
    box(ax, "on");
end

function ConfigureLegend(lgd)

    set(lgd, ...
        "Color", "w", ...
        "TextColor", "k", ...
        "EdgeColor", "k");
end

function y = CleanVector(x)

    y = x(:);
    y = y(~isnan(y));
    y = y(~isinf(y));
end

function value = RMSValue(x)

    x = CleanVector(x);

    if isempty(x)
        value = NaN;
    else
        value = sqrt(mean(x.^2));
    end
end