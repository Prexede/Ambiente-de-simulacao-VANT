function controllersConfig = ControllersConfig(varargin)
% ControllersConfig
% -------------------------------------------------------------------------
% Define os tipos, frequencias e ganhos dos controladores.
%
% Modelos de simulacao:
%   "Normal"    -> posicao P/PD/PID + atitude P/PD/PID
%   "MRAC_Test" -> x/y P/PD/PID + z MRAC + atitude MRAC
% -------------------------------------------------------------------------

    p = inputParser;
    addParameter(p, "SimulationModel", "Normal");
    addParameter(p, "PositionType", "PID");
    addParameter(p, "AttitudeType", "PD");
    addParameter(p, "AltitudeType", "MRAC");
    addParameter(p, "PositionFrequency", 10, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, "AltitudeFrequency", 100, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, "AttitudeFrequency", 100, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parse(p, varargin{:});

    positionGains = PositionGains();
    attitudeGains = AttitudeGains();

    controllersConfig = struct();

    controllersConfig.simulationModel = string(p.Results.SimulationModel);

    controllersConfig.position.type = upper(string(p.Results.PositionType));
    controllersConfig.position.updateFrequency = p.Results.PositionFrequency;
    controllersConfig.position.requestedFrequency = p.Results.PositionFrequency;
    controllersConfig.position.gains = positionGains;

    controllersConfig.altitude.type = upper(string(p.Results.AltitudeType));
    controllersConfig.altitude.updateFrequency = p.Results.AltitudeFrequency;
    controllersConfig.altitude.requestedFrequency = p.Results.AltitudeFrequency;

    controllersConfig.attitude.type = upper(string(p.Results.AttitudeType));
    controllersConfig.attitude.updateFrequency = p.Results.AttitudeFrequency;
    controllersConfig.attitude.requestedFrequency = p.Results.AttitudeFrequency;
    controllersConfig.attitude.gains = attitudeGains;

    if controllersConfig.simulationModel == "MRAC_Test"
        controllersConfig.altitude.type = "MRAC";
        controllersConfig.attitude.type = "MRAC";
    end
end
