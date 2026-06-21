function PlotLapComparison(simData)
% PlotLapComparison
% -------------------------------------------------------------------------
% Compara as voltas sobrepondo os dados de cada volta.
%
% Usa tempo local:
%   tLocal = 0 no inicio de cada volta.
%
% Esta versao mostra:
%   1) erro em x por volta
%   2) erro em y por volta
%   3) erro em z por volta
%   4) norma do erro de posicao por volta
% -------------------------------------------------------------------------

    if ~isfield(simData, "condition") || ~isfield(simData.condition, "lap")
        warning('PlotLapComparison: simData.condition.lap nao encontrado.');
        return;
    end

    t = simData.t(:).';
    lapIndex = simData.condition.lap(:).';

    numLaps = max(lapIndex);

    if numLaps < 2
        warning('PlotLapComparison: ha apenas uma volta para comparar.');
        return;
    end

    if isfield(simData, "error") && isfield(simData.error, "position")
        positionError = simData.error.position;
    else
        positionError = simData.ref.r - simData.state.x(1:3, :);
    end

    errorNorm = vecnorm(positionError, 2, 1);

    colors = lines(numLaps);

    darkBg = [0.10 0.10 0.10];
    darkAxes = [0.08 0.08 0.08];
    textColor = [0.95 0.95 0.95];
    gridColor = [0.35 0.35 0.35];

    fig = figure( ...
        "Name", "Comparacao entre voltas", ...
        "NumberTitle", "off", ...
        "Color", darkBg, ...
        "Tag", "QuadrotorLapComparisonFigure");

    tl = tiledlayout(fig, 2, 2, ...
        "TileSpacing", "compact", ...
        "Padding", "compact");

    title(tl, "Comparacao entre voltas - erro de rastreamento", ...
        "Color", textColor, ...
        "FontWeight", "bold");

    %% Erro em x
    ax1 = nexttile;
    StyleAxes(ax1, darkAxes, textColor, gridColor);
    hold(ax1, "on");
    title(ax1, "Erro em x por volta", "Color", textColor);
    xlabel(ax1, "Tempo local da volta [s]", "Color", textColor);
    ylabel(ax1, "e_x [m]", "Color", textColor);

    PlotLaps(ax1, t, positionError(1,:), lapIndex, numLaps, colors, textColor);

    %% Erro em y
    ax2 = nexttile;
    StyleAxes(ax2, darkAxes, textColor, gridColor);
    hold(ax2, "on");
    title(ax2, "Erro em y por volta", "Color", textColor);
    xlabel(ax2, "Tempo local da volta [s]", "Color", textColor);
    ylabel(ax2, "e_y [m]", "Color", textColor);

    PlotLaps(ax2, t, positionError(2,:), lapIndex, numLaps, colors, textColor);

    %% Erro em z
    ax3 = nexttile;
    StyleAxes(ax3, darkAxes, textColor, gridColor);
    hold(ax3, "on");
    title(ax3, "Erro em z por volta", "Color", textColor);
    xlabel(ax3, "Tempo local da volta [s]", "Color", textColor);
    ylabel(ax3, "e_z [m]", "Color", textColor);

    PlotLaps(ax3, t, positionError(3,:), lapIndex, numLaps, colors, textColor);

    %% Norma do erro
    ax4 = nexttile;
    StyleAxes(ax4, darkAxes, textColor, gridColor);
    hold(ax4, "on");
    title(ax4, "Norma do erro de posicao por volta", "Color", textColor);
    xlabel(ax4, "Tempo local da volta [s]", "Color", textColor);
    ylabel(ax4, "||e_r|| [m]", "Color", textColor);

    PlotLaps(ax4, t, errorNorm, lapIndex, numLaps, colors, textColor);

    linkaxes([ax1, ax2, ax3, ax4], "x");

    PrintLapErrorMetrics(t, positionError, errorNorm, lapIndex, numLaps);
end

% =========================================================================
% Funcoes auxiliares locais
% =========================================================================

function PlotLaps(ax, t, signal, lapIndex, numLaps, colors, textColor)
    for lap = 1:numLaps
        lapMask = lapIndex == lap;

        if ~any(lapMask)
            continue;
        end

        tLap = t(lapMask);
        tLocal = tLap - tLap(1);

        plot(ax, ...
            tLocal, ...
            signal(lapMask), ...
            "-", ...
            "Color", colors(lap, :), ...
            "LineWidth", 1.8, ...
            "DisplayName", sprintf("Volta %d", lap));
    end

    lgd = legend(ax, "Location", "best");
    lgd.TextColor = textColor;
    lgd.Color = [0.14 0.14 0.14];
    lgd.EdgeColor = [0.35 0.35 0.35];
end

function StyleAxes(ax, darkAxes, textColor, gridColor)
    set(ax, ...
        "Color", darkAxes, ...
        "XColor", textColor, ...
        "YColor", textColor, ...
        "GridColor", gridColor, ...
        "MinorGridColor", gridColor, ...
        "Box", "on");

    grid(ax, "on");
end

function PrintLapErrorMetrics(t, positionError, errorNorm, lapIndex, numLaps)
    fprintf('\n--- Comparacao entre voltas: erro de rastreamento ---\n');

    for lap = 1:numLaps
        lapMask = lapIndex == lap;

        if ~any(lapMask)
            continue;
        end

        ex = positionError(1, lapMask);
        ey = positionError(2, lapMask);
        ez = positionError(3, lapMask);
        en = errorNorm(lapMask);

        fprintf('Volta %d:\n', lap);
        fprintf('  RMSE x      = %.6f m\n', sqrt(mean(ex.^2, "omitnan")));
        fprintf('  RMSE y      = %.6f m\n', sqrt(mean(ey.^2, "omitnan")));
        fprintf('  RMSE z      = %.6f m\n', sqrt(mean(ez.^2, "omitnan")));
        fprintf('  RMSE norma  = %.6f m\n', sqrt(mean(en.^2, "omitnan")));
        fprintf('  Max norma   = %.6f m\n', max(en, [], "omitnan"));
    end
end