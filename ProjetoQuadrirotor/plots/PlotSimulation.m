function PlotSimulation(simData, plotConfig)
% PlotSimulation
% -------------------------------------------------------------------------
% Executa os plots selecionados.
% -------------------------------------------------------------------------

    if nargin < 2 || isempty(plotConfig)
        plotConfig = PlotConfig();
    end

    if plotConfig.trajectory
        PlotTrajectory(simData);
    end

    if plotConfig.states
        PlotStates(simData);
    end

    if plotConfig.errors
        PlotErrors(simData);
    end

    if plotConfig.mass
        PlotMassVariation(simData);
    end

    if plotConfig.motors
        PlotMotorSpeeds(simData);
    end

    if isfield(plotConfig, "lapComparison") && plotConfig.lapComparison
        PlotLapComparison(simData);
    end

    drawnow;

    if plotConfig.animation
        AnimateQuadrotorSimulation(simData, plotConfig);
    end
end

