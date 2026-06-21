function PlotTrajectory(simData)
% PlotTrajectory
% -------------------------------------------------------------------------
% Plota a trajetoria 3D desejada e realizada.
%
% Recursos:
%   - Fundo escuro.
%   - Trajetoria separada por volta.
%   - Menu para escolher entre:
%       Todas
%       Volta 1
%       Volta 2
%       ...
% -------------------------------------------------------------------------

    if ~isfield(simData, "state") || ~isfield(simData.state, "x")
        warning("PlotTrajectory: simData.state.x nao encontrado.");
        return;
    end

    if ~isfield(simData, "ref") || ~isfield(simData.ref, "r")
        warning("PlotTrajectory: simData.ref.r nao encontrado.");
        return;
    end

    position = simData.state.x(1:3, :);
    refPosition = simData.ref.r;

    N = size(position, 2);

    if size(refPosition, 2) ~= N
        warning("PlotTrajectory: dimensoes de ref.r e state.x nao coincidem.");
        return;
    end

    if isfield(simData, "condition") && isfield(simData.condition, "lap")
        lapIndex = simData.condition.lap(:).';
    else
        lapIndex = ones(1, N);
    end

    numLaps = max(lapIndex);

    darkBg = [0.10 0.10 0.10];
    darkPanel = [0.14 0.14 0.14];
    darkAxes = [0.08 0.08 0.08];
    textColor = [0.95 0.95 0.95];
    gridColor = [0.35 0.35 0.35];

    colors = lines(numLaps);

    fig = figure( ...
        "Name", "Trajetoria 3D", ...
        "NumberTitle", "off", ...
        "Color", darkBg, ...
        "Tag", "QuadrotorTrajectoryFigure");

    ax = axes( ...
        "Parent", fig, ...
        "Position", [0.07 0.10 0.70 0.82], ...
        "Color", darkAxes, ...
        "XColor", textColor, ...
        "YColor", textColor, ...
        "ZColor", textColor, ...
        "GridColor", gridColor, ...
        "MinorGridColor", gridColor);

    hold(ax, "on");
    grid(ax, "on");
    axis(ax, "equal");
    view(ax, 3);

    xlabel(ax, "x [m]", "Color", textColor);
    ylabel(ax, "y [m]", "Color", textColor);
    zlabel(ax, "z [m]", "Color", textColor);
    title(ax, "Trajetoria desejada vs realizada", "Color", textColor);

    ConfigureAxisLimits(ax, position, refPosition);

    hRef = gobjects(numLaps, 1);
    hReal = gobjects(numLaps, 1);

    for lap = 1:numLaps
        lapMask = lapIndex == lap;

        hRef(lap) = plot3( ...
            ax, ...
            refPosition(1, lapMask), ...
            refPosition(2, lapMask), ...
            refPosition(3, lapMask), ...
            "--", ...
            "Color", colors(lap, :), ...
            "LineWidth", 1.4, ...
            "DisplayName", sprintf("Ref. volta %d", lap));

        hReal(lap) = plot3( ...
            ax, ...
            position(1, lapMask), ...
            position(2, lapMask), ...
            position(3, lapMask), ...
            "-", ...
            "Color", colors(lap, :), ...
            "LineWidth", 1.8, ...
            "DisplayName", sprintf("Real volta %d", lap));
    end

    lapOptions = cell(numLaps + 1, 1);
    lapOptions{1} = "Todas";

    for lap = 1:numLaps
        lapOptions{lap + 1} = sprintf("Volta %d", lap);
    end

    uicontrol( ...
        "Parent", fig, ...
        "Style", "text", ...
        "Units", "normalized", ...
        "Position", [0.80 0.80 0.16 0.04], ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "HorizontalAlignment", "left", ...
        "FontWeight", "bold", ...
        "String", "Mostrar:");

    lapSelector = uicontrol( ...
        "Parent", fig, ...
        "Style", "popupmenu", ...
        "Units", "normalized", ...
        "Position", [0.80 0.75 0.16 0.045], ...
        "String", lapOptions, ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "Callback", @SelectLap);

    RefreshLegend(0);

    % =====================================================================
    % Callback
    % =====================================================================

    function SelectLap(~, ~)
        selectedValue = get(lapSelector, "Value");

        % selectedLap = 0 -> Todas
        % selectedLap = 1 -> Volta 1
        % selectedLap = 2 -> Volta 2
        selectedLap = selectedValue - 1;

        for iLap = 1:numLaps
            if selectedLap == 0 || selectedLap == iLap
                set(hRef(iLap), "Visible", "on");
                set(hReal(iLap), "Visible", "on");
            else
                set(hRef(iLap), "Visible", "off");
                set(hReal(iLap), "Visible", "off");
            end
        end

        if selectedLap == 0
            title(ax, "Trajetoria desejada vs realizada - Todas as voltas", "Color", textColor);
        else
            title(ax, sprintf("Trajetoria desejada vs realizada - Volta %d", selectedLap), "Color", textColor);
        end

        RefreshLegend(selectedLap);
    end

    % =====================================================================
    % Atualiza legenda
    % =====================================================================

    function RefreshLegend(selectedLap)
        legendHandles = gobjects(0);
        legendLabels = {};

        for iLap = 1:numLaps
            if selectedLap == 0 || selectedLap == iLap
                legendHandles(end+1) = hRef(iLap); %#ok<AGROW>
                legendLabels{end+1} = sprintf("Ref. volta %d", iLap); %#ok<AGROW>

                legendHandles(end+1) = hReal(iLap); %#ok<AGROW>
                legendLabels{end+1} = sprintf("Real volta %d", iLap); %#ok<AGROW>
            end
        end

        lgd = legend(ax, legendHandles, legendLabels, "Location", "bestoutside");
        lgd.TextColor = textColor;
        lgd.Color = darkPanel;
        lgd.EdgeColor = gridColor;
    end
end

% =========================================================================
% Funcoes auxiliares locais
% =========================================================================

function ConfigureAxisLimits(ax, position, refPosition)
    allPoints = [position, refPosition];
    validColumns = all(isfinite(allPoints), 1);

    if ~any(validColumns)
        xlim(ax, [-1 1]);
        ylim(ax, [-1 1]);
        zlim(ax, [-1 1]);
        return;
    end

    allPoints = allPoints(:, validColumns);

    minValue = min(allPoints, [], 2);
    maxValue = max(allPoints, [], 2);

    center = 0.5*(minValue + maxValue);
    rangeValue = max(maxValue - minValue);

    if rangeValue <= 0
        rangeValue = 1;
    end

    margin = 0.15*rangeValue;
    halfRange = 0.5*rangeValue + margin;

    xlim(ax, [center(1)-halfRange, center(1)+halfRange]);
    ylim(ax, [center(2)-halfRange, center(2)+halfRange]);
    zlim(ax, [center(3)-halfRange, center(3)+halfRange]);
end