function condition = GenerateDisturbanceProfile(simConfig, quadConfig, disturbanceConfig, lapIndex)
% GenerateDisturbanceProfile
% -------------------------------------------------------------------------
% Avalia os disturbios por volta e gera perfis ponto a ponto.
%
% Modos aceitos:
%   "nominal"    -> sem disturbios
%   "mass"       -> variacao de massa por volta
%   "wind"       -> forca externa por volta
%   "masswind"   -> variacao de massa + forca externa por volta
%
% MassByLap:
%   [volta, delta_massa_kg]
%
% WindByLap:
%   [volta, Fx_N, Fy_N, Fz_N]
% -------------------------------------------------------------------------

    N = numel(simConfig.time.t);
    numLaps = simConfig.time.repetitions;

    mode = lower(string(disturbanceConfig.mode));
    massByLap = disturbanceConfig.massByLap;
    windByLap = disturbanceConfig.windByLap;

    validModes = ["nominal", "mass", "wind", "masswind"];

    if ~any(mode == validModes)
        error('DisturbanceMode invalido. Use "nominal", "mass", "wind" ou "masswind".');
    end

    useMass = mode == "mass" || mode == "masswind";
    useWind = mode == "wind" || mode == "masswind";

    massPerLap = quadConfig.mass*ones(numLaps, 1);
    windPerLap = zeros(numLaps, 3);

    if useMass && ~isempty(massByLap)
        ValidateMassByLap(massByLap);

        for i = 1:size(massByLap, 1)
            lap = round(massByLap(i, 1));

            if lap >= 1 && lap <= numLaps
                deltaMass = massByLap(i, 2);
                massPerLap(lap) = quadConfig.mass + deltaMass;
            end
        end
    end

    if useWind && ~isempty(windByLap)
        ValidateWindByLap(windByLap);

        for i = 1:size(windByLap, 1)
            lap = round(windByLap(i, 1));

            if lap >= 1 && lap <= numLaps
                windPerLap(lap, :) = windByLap(i, 2:4);
            end
        end
    end

    if any(massPerLap <= 0)
        error('A massa resultante precisa ser positiva em todas as voltas.');
    end

    mass = zeros(1, N);
    windInertial = zeros(3, N);

    for k = 1:N
        lap = lapIndex(k);

        mass(k) = massPerLap(lap);
        windInertial(:, k) = windPerLap(lap, :).';
    end

    condition = struct();
    condition.mass = mass;
    condition.windInertial = windInertial;
    condition.forceInertial = windInertial;

    condition.massPerLap = massPerLap;
    condition.windPerLap = windPerLap;
end

function ValidateMassByLap(massByLap)
    if ~isnumeric(massByLap) || size(massByLap, 2) ~= 2
        error('MassByLap deve ter o formato [volta, delta_massa_kg].');
    end
end

function ValidateWindByLap(windByLap)
    if ~isnumeric(windByLap) || size(windByLap, 2) ~= 4
        error('WindByLap deve ter o formato [volta, Fx_N, Fy_N, Fz_N].');
    end
end