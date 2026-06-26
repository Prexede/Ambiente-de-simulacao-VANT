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

adaptationColors = [
    1.0000    0.0000    0.0000;
    0.0000    0.0000    1.0000;
    0.0000    0.6000    0.0000
];

adaptationCases = struct( ...
    "name", {"MRAC lento", "MRAC nominal", "MRAC rapido"}, ...
    "scale", {0.25, 1.00, 500});

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
        1, 0.00; ...
        2, 2; ...
        3, 2], ...
    "WindByLap", [ ...
        1, 0.0, 0.0, 0.0; ...
        2, 0.0, 0.0, 0.0; ...
        3, 0.0, 0.0, 0.0]);

noiseParams = NoiseParams();
noiseParams.enable = false;
noiseParams.disturbanceType = "whiteNoise";

noiseParams.r.enable = true;
noiseParams.r.sigma = [0.0; 0.0; 0.8];

noiseParams.v.enable = true;
noiseParams.v.sigma = [0.00; 0.00; 0.8];

noiseParams.eta.enable = true;
noiseParams.eta.sigma = deg2rad([0.5; 0.5; 0.5]);

noiseParams.omega.enable = true;
noiseParams.omega.sigma = deg2rad([0.2; 0.2; 0.2]);

controllersConfig = ControllersConfig( ...
    "SimulationModel", "MRAC_Test", ...
    "PositionType", "PD", ...
    "AltitudeType", "MRAC", ...
    "AttitudeType", "MRAC", ...
    "PositionFrequency", positionFrequency, ...
    "AltitudeFrequency", altitudeFrequency, ...
    "AttitudeFrequency", attitudeFrequency);

numCases = numel(adaptationCases);
results = struct([]);

fprintf("\n--- Comparacao dos parametros de adaptacao do MRAC ---\n");
fprintf("Trajetoria: %s\n", trajectoryType);
fprintf("Tempo por segmento: %.2f s\n", segmentTime);
fprintf("Repeticoes: %d\n", repetitions);
fprintf("Tempo final: %.2f s\n", simConfig.time.tf);
fprintf("Perturbacao: +2.0 kg nas voltas 2 e 3\n");

for i = 1:numCases
    fprintf("\nRodando caso: %s | escala gamma = %.2f\n", ...
        adaptationCases(i).name, ...
        adaptationCases(i).scale);

    mracParams = MRACParams(quadConfig);
    mracParams = ScaleMRACAdaptation(mracParams, adaptationCases(i).scale);

    simData = SimulationLoop_MRAC_Test( ...
        simConfig, ...
        quadConfig, ...
        controllersConfig, ...
        mracParams, ...
        flightPlan, ...
        noiseParams);

    results(i).name = string(adaptationCases(i).name);
    results(i).scale = adaptationCases(i).scale;
    results(i).color = adaptationColors(i,:);
    results(i).mracParams = mracParams;
    results(i).simData = simData;
end

summaryTable = BuildSummaryTable(results);

fprintf("\n--- Resumo numerico ---\n");
disp(summaryTable);

PlotPositionErrorComparison(results);
PlotAltitudeComparison(results);
PlotAltitudeKdHatComparison(results);
PlotDeltaThrustComparison(results);

function mracParams = ScaleMRACAdaptation(mracParams, scale)

    mracParams.altitude.gammaX = scale*mracParams.altitude.gammaX;
    mracParams.altitude.gammaR = scale*mracParams.altitude.gammaR;
    mracParams.altitude.gammaD = scale*mracParams.altitude.gammaD;
    mracParams.altitude.gammaO = scale*mracParams.altitude.gammaO;

    channels = ["roll", "pitch", "yaw"];

    for i = 1:numel(channels)
        ch = channels(i);

        mracParams.attitude.(ch).gammaX = scale*mracParams.attitude.(ch).gammaX;
        mracParams.attitude.(ch).gammaR = scale*mracParams.attitude.(ch).gammaR;
        mracParams.attitude.(ch).gammaD = scale*mracParams.attitude.(ch).gammaD;
        mracParams.attitude.(ch).gammaO = scale*mracParams.attitude.(ch).gammaO;
    end
end

function summaryTable = BuildSummaryTable(results)

    numCases = numel(results);

    Configuracao = strings(numCases, 1);
    Escala_Gamma = zeros(numCases, 1);
    RMSE_Position_m = zeros(numCases, 1);
    Max_Position_Error_m = zeros(numCases, 1);
    RMSE_Attitude_deg = zeros(numCases, 1);
    Max_Attitude_Error_deg = zeros(numCases, 1);
    Final_KdHat_Altitude = zeros(numCases, 1);
    MaxAbs_KdHat_Altitude = zeros(numCases, 1);

    for i = 1:numCases
        simData = results(i).simData;

        ePos = vecnorm(simData.error.position, 2, 1);
        eAtt = rad2deg(vecnorm(simData.error.attitude, 2, 1));
        kdHat = simData.mrac.altitude.KdHat;

        ePos = CleanVector(ePos);
        eAtt = CleanVector(eAtt);
        kdHatClean = CleanVector(kdHat);

        Configuracao(i) = results(i).name;
        Escala_Gamma(i) = results(i).scale;
        RMSE_Position_m(i) = RMSValue(ePos);
        Max_Position_Error_m(i) = max(ePos);
        RMSE_Attitude_deg(i) = RMSValue(eAtt);
        Max_Attitude_Error_deg(i) = max(eAtt);

        if isempty(kdHatClean)
            Final_KdHat_Altitude(i) = NaN;
            MaxAbs_KdHat_Altitude(i) = NaN;
        else
            Final_KdHat_Altitude(i) = kdHatClean(end);
            MaxAbs_KdHat_Altitude(i) = max(abs(kdHatClean));
        end
    end

    summaryTable = table( ...
        Configuracao, ...
        Escala_Gamma, ...
        RMSE_Position_m, ...
        Max_Position_Error_m, ...
        RMSE_Attitude_deg, ...
        Max_Attitude_Error_deg, ...
        Final_KdHat_Altitude, ...
        MaxAbs_KdHat_Altitude);
end

function PlotPositionErrorComparison(results)

    fig = figure("Name", "Comparacao_MRAC_Erro_Posicao", ...
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

    AddLapMarkers(ax, results(1).simData);
    ConfigureAxes(ax, false);

    xlabel(ax, "Tempo [s]", "Color", "k");
    ylabel(ax, "||e_r|| [m]", "Color", "k");
    title(ax, "Erro de posicao", "Color", "k");

    lgd = legend(ax, "show", "Location", "best");
    ConfigureLegend(lgd);
end

function PlotAltitudeComparison(results)

    idx = StateIndex();

    fig = figure("Name", "Comparacao_MRAC_Altitude", ...
        "NumberTitle", "off", ...
        "Color", "w", ...
        "InvertHardcopy", "off");

    ax = axes("Parent", fig);
    hold(ax, "on");

    simDataRef = results(1).simData;
    t = simDataRef.t;
    zRef = simDataRef.ref.r(3,:);

    plot(ax, t, zRef, ...
        "--", ...
        "Color", [0 0 0], ...
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

    AddLapMarkers(ax, results(1).simData);
    ConfigureAxes(ax, false);

    xlabel(ax, "Tempo [s]", "Color", "k");
    ylabel(ax, "z [m]", "Color", "k");
    title(ax, "Rastreamento de altitude", "Color", "k");

    lgd = legend(ax, "show", "Location", "best");
    ConfigureLegend(lgd);
end

function PlotAltitudeKdHatComparison(results)

    fig = figure("Name", "Comparacao_MRAC_KdHat_Altitude", ...
        "NumberTitle", "off", ...
        "Color", "w", ...
        "InvertHardcopy", "off");

    ax = axes("Parent", fig);
    hold(ax, "on");

    for i = 1:numel(results)
        simData = results(i).simData;
        t = simData.t;
        kdHat = simData.mrac.altitude.KdHat;

        plot(ax, t, kdHat, ...
            "Color", results(i).color, ...
            "LineWidth", 1.5, ...
            "DisplayName", results(i).name);
    end

    AddLapMarkers(ax, results(1).simData);
    ConfigureAxes(ax, false);

    xlabel(ax, "Tempo [s]", "Color", "k");
    ylabel(ax, "\hat{K}_d altitude", "Color", "k");
    title(ax, "Parametro adaptativo de altitude", "Color", "k");

    lgd = legend(ax, "show", "Location", "best");
    ConfigureLegend(lgd);
end

function PlotDeltaThrustComparison(results)

    fig = figure("Name", "Comparacao_MRAC_Delta_Empuxo", ...
        "NumberTitle", "off", ...
        "Color", "w", ...
        "InvertHardcopy", "off");

    ax = axes("Parent", fig);
    hold(ax, "on");

    for i = 1:numel(results)
        simData = results(i).simData;
        t = simData.t;
        deltaThrust = simData.mrac.altitude.deltaThrust;

        plot(ax, t, deltaThrust, ...
            "Color", results(i).color, ...
            "LineWidth", 1.5, ...
            "DisplayName", results(i).name);
    end

    AddLapMarkers(ax, results(1).simData);
    ConfigureAxes(ax, false);

    xlabel(ax, "Tempo [s]", "Color", "k");
    ylabel(ax, "\Delta T_{MRAC} [N]", "Color", "k");
    title(ax, "Empuxo adaptativo incremental", "Color", "k");

    lgd = legend(ax, "show", "Location", "best");
    ConfigureLegend(lgd);
end

function AddLapMarkers(ax, simData)

    t = simData.t;
    lap = simData.condition.lap;

    changeIdx = find(diff(lap) ~= 0) + 1;

    for i = 1:numel(changeIdx)
        xline(ax, t(changeIdx(i)), ...
            "--", ...
            "Color", [0.45 0.45 0.45], ...
            "LineWidth", 1.2, ...
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