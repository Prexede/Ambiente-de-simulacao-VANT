function controllersConfig = ControllersConfig(varargin)
% ControllersConfig
% -------------------------------------------------------------------------
% Define os tipos, frequencias e ganhos dos controladores de posicao e
% atitude.
%
% Os ganhos sempre sao carregados dos arquivos:
%   PositionGains.m
%   AttitudeGains.m
% -------------------------------------------------------------------------

    p = inputParser;
    addParameter(p, "PositionType", "PID");
    addParameter(p, "AttitudeType", "PD");
    addParameter(p, "PositionFrequency", 10, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, "AttitudeFrequency", 100, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parse(p, varargin{:});

    positionGains = PositionGains();
    attitudeGains = AttitudeGains();

    controllersConfig = struct();

    controllersConfig.position.type = upper(string(p.Results.PositionType));
    controllersConfig.position.updateFrequency = p.Results.PositionFrequency;
    controllersConfig.position.requestedFrequency = p.Results.PositionFrequency;
    controllersConfig.position.gains = positionGains;

    controllersConfig.attitude.type = upper(string(p.Results.AttitudeType));
    controllersConfig.attitude.updateFrequency = p.Results.AttitudeFrequency;
    controllersConfig.attitude.requestedFrequency = p.Results.AttitudeFrequency;
    controllersConfig.attitude.gains = attitudeGains;

    ValidateControllerType(controllersConfig.position.type, "position");
    ValidateControllerType(controllersConfig.attitude.type, "attitude");
end

function ValidateControllerType(controllerType, controllerName)
    validTypes = ["P", "PD", "PID"];

    if ~any(controllerType == validTypes)
        error('Controlador de %s invalido. Use "P", "PD" ou "PID".', controllerName);
    end
end