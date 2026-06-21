function plotConfig = PlotConfig(varargin)
% PlotConfig
% -------------------------------------------------------------------------
% Configura quais graficos serao exibidos.
% -------------------------------------------------------------------------

    p = inputParser;

    addParameter(p, "Trajectory", true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, "States", true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, "Errors", true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, "Mass", true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, "Motors", true, @(x) islogical(x) || isnumeric(x));

    addParameter(p, "Animation", false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, "AnimationStep", 10, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, "AnimationPeriod", 0.03, @(x) isnumeric(x) && isscalar(x) && x > 0);

    parse(p, varargin{:});

    plotConfig = struct();

    plotConfig.trajectory = logical(p.Results.Trajectory);
    plotConfig.states = logical(p.Results.States);
    plotConfig.errors = logical(p.Results.Errors);
    plotConfig.mass = logical(p.Results.Mass);
    plotConfig.motors = logical(p.Results.Motors);

    plotConfig.animation = logical(p.Results.Animation);
    plotConfig.animationStep = round(p.Results.AnimationStep);
    plotConfig.animationPeriod = p.Results.AnimationPeriod;
end