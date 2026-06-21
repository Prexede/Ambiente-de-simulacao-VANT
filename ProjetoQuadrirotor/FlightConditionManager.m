function [quadPlant, disturbance, lapIndex] = FlightConditionManager( ...
    quadNominal, tNow, traj, trajConfig, flightConfig)
%FLIGHTCONDITIONMANAGER Aplica condicoes de voo por volta.
%
% Nesta versao, a tabela de massa e interpretada APENAS como variacao
% em relacao a massa nominal:
%
%   massa_real = massa_nominal + delta_massa
%
% Formato esperado:
%
%   flightConfig.mass.byLap = [
%       volta, delta_massa_kg
%   ];
%
% A tabela de vento e interpretada como forca externa aplicada na planta,
% somada na dinamica translacional:
%
%   F_total = F_modelo + F_vento
%
% Formato esperado:
%
%   flightConfig.wind.byLap = [
%       volta, Fx_N, Fy_N, Fz_N
%   ];
%
% Saidas:
%   quadPlant   : parametros reais da planta no instante atual
%   disturbance : estrutura de perturbacoes externas
%   lapIndex    : volta atual

    %% Valores padrao
    quadPlant = quadNominal;

    disturbance = struct();
    disturbance.forceInertial = zeros(3,1);  % [N]
    disturbance.forceBody     = zeros(3,1);  % [N]
    disturbance.torqueBody    = zeros(3,1);  % [N.m]

    if nargin < 5 || isempty(flightConfig)
        flightConfig = struct();
        flightConfig.mode = "Nominal";
    end

    mode = NormalizeMode(GetFlightMode(flightConfig));

    %% Volta atual
    numLaps = GetNumberOfLaps(trajConfig, flightConfig);
    lapDuration = GetLapDuration(traj, trajConfig, flightConfig, numLaps);

    if isinf(lapDuration) || lapDuration <= 0
        lapIndex = 1;
    else
        t0 = GetTrajectoryStartTime(traj);
        lapIndex = floor((tNow - t0)/lapDuration) + 1;
        lapIndex = max(1, lapIndex);
        lapIndex = min(numLaps, lapIndex);
    end

    %% Mudanca de massa por DELTA
    if ModeUsesMass(mode)
        massEnabled = GetNestedLogical(flightConfig, 'mass', 'enabled', true);

        if massEnabled
            mNominal = GetQuadMass(quadNominal);
            deltaMass = GetDeltaMassByLap(flightConfig, lapIndex);
            mReal = mNominal + deltaMass;

            if ~isnumeric(mReal) || ~isscalar(mReal) || ~isfinite(mReal) || mReal <= 0
                error(['Massa real invalida na volta %d. ', ...
                       'Verifique flightConfig.mass.byLap. ', ...
                       'A massa real e calculada como massa_nominal + delta_massa.'], lapIndex);
            end

            quadPlant = SetQuadMass(quadPlant, mReal);
        end
    end

    %% Forca de vento aplicada na planta
    if ModeUsesWind(mode)
        windEnabled = GetNestedLogical(flightConfig, 'wind', 'enabled', true);

        if windEnabled
            disturbance.forceInertial = GetWindForceByLap(flightConfig, lapIndex);
        end
    end
end

%% ========================================================================
% Funcoes auxiliares
% ========================================================================

function mode = GetFlightMode(flightConfig)
    mode = "Nominal";

    if ~isstruct(flightConfig)
        return;
    end

    if isfield(flightConfig, 'mode')
        mode = string(flightConfig.mode);
    elseif isfield(flightConfig, 'flightMode')
        mode = string(flightConfig.flightMode);
    end
end

function mode = NormalizeMode(modeIn)
    txt = lower(char(string(modeIn)));
    txt = erase(txt, ' ');
    txt = erase(txt, '_');
    txt = erase(txt, '-');

    switch txt
        case {'nominal', 'modeloatual', 'atual'}
            mode = "nominal";

        case {'trajectoryonly', 'trajectory', 'trajetoria', ...
              'sotrajetoria', 'somente trajetoria', 'somente_trajetoria'}
            mode = "trajectoryonly";

        case {'masschange', 'mass', 'massa', 'mudancamassa', ...
              'mudancademassa', 'massachange'}
            mode = "masschange";

        case {'wind', 'vento'}
            mode = "wind";

        case {'masswind', 'windmass', 'massavento', 'ventomassa', ...
              'massandwind', 'massaevento'}
            mode = "masswind";

        otherwise
            error('Modo de voo nao reconhecido: %s', char(string(modeIn)));
    end
end

function flag = ModeUsesMass(mode)
    flag = strcmp(mode, "masschange") || strcmp(mode, "masswind");
end

function flag = ModeUsesWind(mode)
    flag = strcmp(mode, "wind") || strcmp(mode, "masswind");
end

function numLaps = GetNumberOfLaps(trajConfig, flightConfig)
    numLaps = 1;

    candidates = {'repetitions', 'Repetitions', 'numRepetitions', ...
                  'NumRepetitions', 'nRepetitions', 'laps', ...
                  'numLaps', 'Nlaps', 'voltas', 'numVoltas'};

    numLaps = GetFirstPositiveScalarField(trajConfig, candidates, numLaps);
    numLaps = GetFirstPositiveScalarField(flightConfig, candidates, numLaps);

    maxLapFromTables = 1;

    if isstruct(flightConfig)
        if isfield(flightConfig, 'mass') && isstruct(flightConfig.mass) && ...
           isfield(flightConfig.mass, 'byLap') && isnumeric(flightConfig.mass.byLap) && ...
           size(flightConfig.mass.byLap,2) >= 1
            maxLapFromTables = max(maxLapFromTables, max(flightConfig.mass.byLap(:,1)));
        end

        if isfield(flightConfig, 'wind') && isstruct(flightConfig.wind) && ...
           isfield(flightConfig.wind, 'byLap') && isnumeric(flightConfig.wind.byLap) && ...
           size(flightConfig.wind.byLap,2) >= 1
            maxLapFromTables = max(maxLapFromTables, max(flightConfig.wind.byLap(:,1)));
        end
    end

    numLaps = max(round(numLaps), round(maxLapFromTables));
    numLaps = max(1, numLaps);
end

function value = GetFirstPositiveScalarField(config, fieldNames, defaultValue)
    value = defaultValue;

    if ~isstruct(config)
        return;
    end

    for i = 1:numel(fieldNames)
        fieldName = fieldNames{i};

        if isfield(config, fieldName)
            candidate = config.(fieldName);

            if isnumeric(candidate) && isscalar(candidate) && isfinite(candidate) && candidate > 0
                value = candidate;
                return;
            end
        end
    end
end

function lapDuration = GetLapDuration(traj, trajConfig, flightConfig, numLaps)
    lapDuration = inf;

    directFields = {'lapDuration', 'LapDuration', 'tempoVolta', 'TempoVolta'};

    lapDuration = GetFirstPositiveScalarField(flightConfig, directFields, lapDuration);
    if isfinite(lapDuration)
        return;
    end

    lapDuration = GetFirstPositiveScalarField(trajConfig, directFields, lapDuration);
    if isfinite(lapDuration)
        return;
    end

    if isstruct(traj) && isfield(traj, 't') && isnumeric(traj.t)
        tVec = traj.t(:);

        if numel(tVec) >= 2
            totalTime = max(tVec) - min(tVec);

            if totalTime > 0
                lapDuration = totalTime/max(1, numLaps);
                return;
            end
        end
    end

    tempoSegmento = [];
    waypoints = [];

    if isstruct(trajConfig)
        if isfield(trajConfig, 'TempoSegmento')
            tempoSegmento = trajConfig.TempoSegmento;
        elseif isfield(trajConfig, 'tempoSegmento')
            tempoSegmento = trajConfig.tempoSegmento;
        elseif isfield(trajConfig, 'segmentTime')
            tempoSegmento = trajConfig.segmentTime;
        end

        if isfield(trajConfig, 'waypoints')
            waypoints = trajConfig.waypoints;
        elseif isfield(trajConfig, 'Waypoints')
            waypoints = trajConfig.Waypoints;
        end
    end

    if isnumeric(tempoSegmento) && isscalar(tempoSegmento) && tempoSegmento > 0 && ...
       isnumeric(waypoints) && size(waypoints,1) >= 2
        lapDuration = tempoSegmento*(size(waypoints,1) - 1);
    end
end

function t0 = GetTrajectoryStartTime(traj)
    t0 = 0;

    if isstruct(traj) && isfield(traj, 't') && isnumeric(traj.t) && ~isempty(traj.t)
        t0 = min(traj.t(:));
    end
end

function m = GetQuadMass(quad)
    if isstruct(quad) && isfield(quad, 'mass')
        m = quad.mass;
    elseif isstruct(quad) && isfield(quad, 'm')
        m = quad.m;
    else
        error('Campo de massa nao encontrado. Use quad.mass ou quad.m.');
    end
end

function quad = SetQuadMass(quad, mNew)
    if isfield(quad, 'mass')
        quad.mass = mNew;
    end

    if isfield(quad, 'm')
        quad.m = mNew;
    end

    if ~isfield(quad, 'mass') && ~isfield(quad, 'm')
        quad.mass = mNew;
    end
end

function deltaMass = GetDeltaMassByLap(flightConfig, lapIndex)
    deltaMass = 0;

    if ~isstruct(flightConfig) || ~isfield(flightConfig, 'mass') || ...
       ~isstruct(flightConfig.mass) || ~isfield(flightConfig.mass, 'byLap')
        error(['Para usar mudanca de massa, defina: ', ...
               'flightConfig.mass.byLap = [volta, delta_massa_kg].']);
    end

    massTable = flightConfig.mass.byLap;

    if ~isnumeric(massTable) || size(massTable,2) ~= 2
        error('flightConfig.mass.byLap deve ter exatamente 2 colunas: [volta, delta_massa_kg].');
    end

    idx = find(massTable(:,1) <= lapIndex, 1, 'last');

    if isempty(idx)
        idx = 1;
    end

    deltaMass = massTable(idx,2);
end

function F = GetWindForceByLap(flightConfig, lapIndex)
    F = zeros(3,1);

    if ~isstruct(flightConfig) || ~isfield(flightConfig, 'wind') || ...
       ~isstruct(flightConfig.wind) || ~isfield(flightConfig.wind, 'byLap')
        error(['Para usar vento, defina: ', ...
               'flightConfig.wind.byLap = [volta, Fx_N, Fy_N, Fz_N].']);
    end

    windTable = flightConfig.wind.byLap;

    if ~isnumeric(windTable) || size(windTable,2) ~= 4
        error('flightConfig.wind.byLap deve ter exatamente 4 colunas: [volta, Fx_N, Fy_N, Fz_N].');
    end

    idx = find(windTable(:,1) <= lapIndex, 1, 'last');

    if isempty(idx)
        idx = 1;
    end

    F = windTable(idx,2:4).';
end

function value = GetNestedLogical(config, parentField, childField, defaultValue)
    value = defaultValue;

    if ~isstruct(config) || ~isfield(config, parentField)
        return;
    end

    parent = config.(parentField);

    if ~isstruct(parent) || ~isfield(parent, childField)
        return;
    end

    candidate = parent.(childField);

    if islogical(candidate) && isscalar(candidate)
        value = candidate;
    elseif isnumeric(candidate) && isscalar(candidate)
        value = candidate ~= 0;
    elseif ischar(candidate) || isstring(candidate)
        txt = lower(char(string(candidate)));
        value = strcmp(txt, 'true') || strcmp(txt, 'on') || ...
                strcmp(txt, 'yes') || strcmp(txt, 'sim') || strcmp(txt, '1');
    end
end
