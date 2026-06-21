function [posGains, attGains, gainInfo] = GetControllerGains(quad, controlConfig)
% GetControllerGains
% -------------------------------------------------------------------------
% Decide a origem dos ganhos dos controladores.
%
% Modos:
%   controlConfig.gainSource = "arquivo"
%       Usa PositionPIDGains.m e AttitudePIDGains.m
%
%   controlConfig.gainSource = "estimador"
%       Calcula ganhos PD/PID usando tempo de pico e tempo de estabilizacao.
%
% O objetivo deste arquivo e manter o main.m simples. O main escolhe apenas
% a origem dos ganhos; esta funcao decide qual arquivo/função deve chamar.
% -------------------------------------------------------------------------

    if nargin < 2 || isempty(controlConfig)
        error('Informe controlConfig.');
    end

    if ~isfield(controlConfig, 'gainSource') || isempty(controlConfig.gainSource)
        controlConfig.gainSource = "arquivo";
    end

    gainSource = lower(strtrim(string(controlConfig.gainSource)));

    switch gainSource

        case {"arquivo", "file", "direto", "direct", "manual"}

            posGains = PositionPIDGains();
            attGains = AttitudePIDGains();

            gainInfo = struct();
            gainInfo.source = "arquivo";
            gainInfo.description = ...
                "Ganhos carregados de PositionPIDGains.m e AttitudePIDGains.m.";
            gainInfo.position = posGains;
            gainInfo.attitude = attGains;

        case {"estimador", "estimate", "estimated", "spec", "specs"}

            if ~isfield(controlConfig, 'gainSpec') || isempty(controlConfig.gainSpec)
                error('Para gainSource = "estimador", defina controlConfig.gainSpec.');
            end

            if ~isfield(controlConfig, 'settlingFactor') || isempty(controlConfig.settlingFactor)
                controlConfig.settlingFactor = 4.0;
            end

            if ~isfield(controlConfig, 'pidGamma') || isempty(controlConfig.pidGamma)
                controlConfig.pidGamma = 5;
            end

            [posGains, attGains, gainInfo] = EstimateControllerGains( ...
                quad, ...
                controlConfig.gainSpec, ...
                controlConfig.settlingFactor, ...
                controlConfig.pidGamma);

            gainInfo.source = "estimador";

        otherwise

            error('controlConfig.gainSource invalido. Use "arquivo" ou "estimador".');
    end
end
