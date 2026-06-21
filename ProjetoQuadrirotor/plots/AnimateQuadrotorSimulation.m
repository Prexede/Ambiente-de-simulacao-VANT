function anim = AnimateQuadrotorSimulation(simData, plotConfig)
% AnimateQuadrotorSimulation
% -------------------------------------------------------------------------
% Animacao 3D nao bloqueante da simulacao do quadrotor.
%
% Recursos:
%   - Figura propria para animacao.
%   - Fundo escuro.
%   - Timer para nao travar o MATLAB.
%   - Botoes para pausar, parar e reiniciar.
%   - Menu para escolher qual volta mostrar.
%   - Painel de informacoes do drone.
%   - Trajetoria de cada volta com cor diferente.
%   - Garante que apenas uma animacao fique aberta por vez.
% -------------------------------------------------------------------------

    if nargin < 2 || isempty(plotConfig)
        plotConfig = PlotConfig("Animation", true);
    end

    % ---------------------------------------------------------------------
    % Fecha qualquer animacao anterior para evitar abrir duas janelas
    % ---------------------------------------------------------------------
    existingFig = findall(0, "Type", "figure", "Tag", "QuadrotorAnimationFigure");

    for i = 1:numel(existingFig)
        try
            if isappdata(existingFig(i), "AnimationTimer")
                oldTimer = getappdata(existingFig(i), "AnimationTimer");
                SafeStopAndDeleteTimer(oldTimer);
            end
            if isvalid(existingFig(i))
                delete(existingFig(i));
            end
        catch
        end
    end

    t = simData.t(:).';
    N = numel(t);

    if N < 2
        warning('Nao ha amostras suficientes para animar.');
        anim = [];
        return;
    end

    if isfield(plotConfig, "animationStep")
        frameStep = plotConfig.animationStep;
    else
        frameStep = 10;
    end

    if isfield(plotConfig, "animationPeriod")
        animationPeriod = plotConfig.animationPeriod;
    else
        animationPeriod = 0.03;
    end

    frameStep = max(1, round(frameStep));

    state = simData.state.x;

    position = state(1:3, :);
    velocity = state(4:6, :);
    attitude = state(7:9, :);

    refPosition = simData.ref.r;

    if isfield(simData, "condition") && isfield(simData.condition, "lap")
        lapIndex = simData.condition.lap(:).';
    else
        lapIndex = ones(1, N);
    end

    if isfield(simData, "condition") && isfield(simData.condition, "segmentInLap")
        segmentIndex = simData.condition.segmentInLap(:).';
    elseif isfield(simData, "condition") && isfield(simData.condition, "segment")
        segmentIndex = simData.condition.segment(:).';
    else
        segmentIndex = ones(1, N);
    end

    if isfield(simData, "condition") && isfield(simData.condition, "mass")
        massHistory = simData.condition.mass(:).';
    else
        massHistory = nan(1, N);
    end

    if isfield(simData, "cmd") && isfield(simData.cmd, "motorOmega")
        motorOmega = simData.cmd.motorOmega;
    else
        motorOmega = nan(4, N);
    end

    numLaps = max(lapIndex);
    numSegmentsPerLap = max(segmentIndex);

    selectedLap = 0; % 0 = todas as voltas
    frameList = BuildFrameList(selectedLap);

    colors = lines(numLaps);

    darkBg = [0.10 0.10 0.10];
    darkPanel = [0.14 0.14 0.14];
    darkAxes = [0.08 0.08 0.08];
    textColor = [0.95 0.95 0.95];
    gridColor = [0.35 0.35 0.35];

    fig = figure( ...
        "Name", "Animacao da simulacao do quadrotor", ...
        "NumberTitle", "off", ...
        "Color", darkBg, ...
        "WindowStyle", "normal", ...
        "Tag", "QuadrotorAnimationFigure", ...
        "CloseRequestFcn", @CloseAnimation);

    ax = axes( ...
        "Parent", fig, ...
        "Position", [0.06 0.10 0.62 0.84], ...
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
    title(ax, "Animacao 3D do quadrotor", "Color", textColor);

    ConfigureAxisLimits(ax, position, refPosition);

    hRef = gobjects(numLaps, 1);
    hActual = gobjects(numLaps, 1);

    for lap = 1:numLaps
        lapMask = lapIndex == lap;

        hRef(lap) = plot3( ...
            ax, ...
            refPosition(1, lapMask), ...
            refPosition(2, lapMask), ...
            refPosition(3, lapMask), ...
            "--", ...
            "Color", colors(lap, :), ...
            "LineWidth", 1.2, ...
            "DisplayName", sprintf("Ref. volta %d", lap));

        hActual(lap) = plot3( ...
            ax, ...
            nan, nan, nan, ...
            "-", ...
            "Color", colors(lap, :), ...
            "LineWidth", 2.0, ...
            "DisplayName", sprintf("Real volta %d", lap));
    end

    lgd = legend(ax, "Location", "bestoutside");
    lgd.TextColor = textColor;
    lgd.Color = darkPanel;
    lgd.EdgeColor = gridColor;

    armVisualLength = GetVisualArmLength(simData, position, refPosition);

    hArmX = plot3(ax, nan, nan, nan, "-", ...
        "Color", textColor, ...
        "LineWidth", 3, ...
        "HandleVisibility", "off");

    hArmY = plot3(ax, nan, nan, nan, "-", ...
        "Color", textColor, ...
        "LineWidth", 3, ...
        "HandleVisibility", "off");

    hMotors = plot3(ax, nan, nan, nan, "o", ...
        "Color", textColor, ...
        "MarkerFaceColor", textColor, ...
        "MarkerSize", 6, ...
        "HandleVisibility", "off");

    hCenter = plot3(ax, nan, nan, nan, "o", ...
        "Color", textColor, ...
        "MarkerFaceColor", darkAxes, ...
        "MarkerSize", 8, ...
        "HandleVisibility", "off");

    hHeading = plot3(ax, nan, nan, nan, "-", ...
        "Color", textColor, ...
        "LineWidth", 1.5, ...
        "HandleVisibility", "off");

    infoBox = uicontrol( ...
        "Parent", fig, ...
        "Style", "text", ...
        "Units", "normalized", ...
        "Position", [0.71 0.43 0.27 0.50], ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "HorizontalAlignment", "left", ...
        "FontName", "Consolas", ...
        "FontSize", 10, ...
        "String", "");

    statusBox = uicontrol( ...
        "Parent", fig, ...
        "Style", "text", ...
        "Units", "normalized", ...
        "Position", [0.71 0.35 0.27 0.06], ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "HorizontalAlignment", "left", ...
        "FontWeight", "bold", ...
        "String", "Status: rodando");

    lapOptions = cell(numLaps + 1, 1);
    lapOptions{1} = 'Todas';

    for lap = 1:numLaps
        lapOptions{lap + 1} = sprintf('Volta %d', lap);
    end

    lapLabel = uicontrol( ...
        "Parent", fig, ...
        "Style", "text", ...
        "Units", "normalized", ...
        "Position", [0.71 0.30 0.10 0.04], ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "HorizontalAlignment", "left", ...
        "FontWeight", "bold", ...
        "String", "Mostrar:");

    lapSelector = uicontrol( ...
        "Parent", fig, ...
        "Style", "popupmenu", ...
        "Units", "normalized", ...
        "Position", [0.82 0.30 0.16 0.045], ...
        "String", lapOptions, ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "Callback", @SelectLap);

    pauseButton = uicontrol( ...
        "Parent", fig, ...
        "Style", "pushbutton", ...
        "Units", "normalized", ...
        "Position", [0.71 0.22 0.12 0.06], ...
        "String", "Pausar", ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "Callback", @TogglePause);

    stopButton = uicontrol( ...
        "Parent", fig, ...
        "Style", "pushbutton", ...
        "Units", "normalized", ...
        "Position", [0.86 0.22 0.12 0.06], ...
        "String", "Parar", ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "Callback", @StopAnimation);

    restartButton = uicontrol( ...
        "Parent", fig, ...
        "Style", "pushbutton", ...
        "Units", "normalized", ...
        "Position", [0.71 0.14 0.27 0.06], ...
        "String", "Reiniciar", ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "Callback", @RestartAnimation);

    animationTimer = timer( ...
        "ExecutionMode", "fixedSpacing", ...
        "Period", animationPeriod, ...
        "BusyMode", "drop", ...
        "TimerFcn", @UpdateAnimation);

    animationState = struct();
    animationState.framePointer = 1;
    animationState.frameList = frameList;
    animationState.selectedLap = selectedLap;
    animationState.isPaused = false;
    animationState.isStopped = false;
    animationState.timer = animationTimer;

    animationState.handles.ax = ax;
    animationState.handles.hRef = hRef;
    animationState.handles.hActual = hActual;
    animationState.handles.hArmX = hArmX;
    animationState.handles.hArmY = hArmY;
    animationState.handles.hMotors = hMotors;
    animationState.handles.hCenter = hCenter;
    animationState.handles.hHeading = hHeading;
    animationState.handles.infoBox = infoBox;
    animationState.handles.statusBox = statusBox;
    animationState.handles.pauseButton = pauseButton;
    animationState.handles.lapSelector = lapSelector;
    animationState.handles.lapLabel = lapLabel;

    setappdata(fig, "AnimationTimer", animationTimer);
    set(fig, "UserData", animationState);

    UpdateFrame(1);
    start(animationTimer);

    if nargout > 0
        anim.figure = fig;
        anim.timer = animationTimer;
    end

    % =====================================================================
    % Funcoes internas
    % =====================================================================

    function frames = BuildFrameList(lapToShow)
        if lapToShow == 0
            frames = 1:frameStep:N;

            if frames(end) ~= N
                frames(end+1) = N;
            end

            return;
        end

        lapFrames = find(lapIndex == lapToShow);

        if isempty(lapFrames)
            frames = [];
            return;
        end

        frames = lapFrames(1):frameStep:lapFrames(end);

        if frames(end) ~= lapFrames(end)
            frames(end+1) = lapFrames(end);
        end
    end

    % =====================================================================
    % Callbacks
    % =====================================================================

    function UpdateAnimation(~, ~)
        if ~ishandle(fig)
            SafeStopAndDeleteTimer(animationTimer);
            return;
        end

        data = get(fig, "UserData");

        if data.isStopped
            SafeStopTimer(animationTimer);
            return;
        end

        if data.isPaused
            return;
        end

        if isempty(data.frameList)
            SafeStopTimer(animationTimer);
            set(data.handles.statusBox, "String", "Status: sem dados");
            return;
        end

        if data.framePointer > numel(data.frameList)
            SafeStopTimer(animationTimer);
            set(data.handles.statusBox, "String", "Status: finalizada");
            return;
        end

        k = data.frameList(data.framePointer);

        UpdateFrame(k);

        if data.framePointer < numel(data.frameList)
            data.framePointer = data.framePointer + 1;
            set(fig, "UserData", data);
        else
            SafeStopTimer(animationTimer);
            set(data.handles.statusBox, "String", "Status: finalizada");
        end
    end

    function SelectLap(~, ~)
        if ~ishandle(fig)
            return;
        end

        data = get(fig, "UserData");

        selectedValue = get(data.handles.lapSelector, "Value");
        data.selectedLap = selectedValue - 1;

        data.frameList = BuildFrameList(data.selectedLap);
        data.framePointer = 1;
        data.isPaused = false;
        data.isStopped = false;

        for lap = 1:numLaps
            set(data.handles.hActual(lap), ...
                "XData", nan, ...
                "YData", nan, ...
                "ZData", nan);

            if data.selectedLap == 0 || data.selectedLap == lap
                set(data.handles.hRef(lap), "Visible", "on");
                set(data.handles.hActual(lap), "Visible", "on");
            else
                set(data.handles.hRef(lap), "Visible", "off");
                set(data.handles.hActual(lap), "Visible", "off");
            end
        end

        set(data.handles.pauseButton, "String", "Pausar");
        set(data.handles.statusBox, "String", "Status: rodando");

        set(fig, "UserData", data);

        if ~isempty(data.frameList)
            UpdateFrame(data.frameList(1));
        else
            set(data.handles.statusBox, "String", "Status: sem dados");
        end

        if strcmp(animationTimer.Running, "off") && ~isempty(data.frameList)
            start(animationTimer);
        end
    end

    function TogglePause(~, ~)
        if ~ishandle(fig)
            return;
        end

        data = get(fig, "UserData");
        data.isPaused = ~data.isPaused;

        if data.isPaused
            set(data.handles.pauseButton, "String", "Continuar");
            set(data.handles.statusBox, "String", "Status: pausada");
        else
            set(data.handles.pauseButton, "String", "Pausar");
            set(data.handles.statusBox, "String", "Status: rodando");
        end

        set(fig, "UserData", data);
    end

    function StopAnimation(~, ~)
        if ~ishandle(fig)
            return;
        end

        data = get(fig, "UserData");
        data.isStopped = true;

        set(data.handles.statusBox, "String", "Status: parada");
        set(fig, "UserData", data);

        SafeStopTimer(animationTimer);
    end

    function RestartAnimation(~, ~)
        if ~ishandle(fig)
            return;
        end

        data = get(fig, "UserData");

        data.frameList = BuildFrameList(data.selectedLap);
        data.framePointer = 1;
        data.isPaused = false;
        data.isStopped = false;

        for lap = 1:numLaps
            set(data.handles.hActual(lap), ...
                "XData", nan, ...
                "YData", nan, ...
                "ZData", nan);
        end

        set(data.handles.pauseButton, "String", "Pausar");
        set(data.handles.statusBox, "String", "Status: rodando");
        set(fig, "UserData", data);

        if ~isempty(data.frameList)
            UpdateFrame(data.frameList(1));
        else
            set(data.handles.statusBox, "String", "Status: sem dados");
        end

        if strcmp(animationTimer.Running, "off") && ~isempty(data.frameList)
            start(animationTimer);
        end
    end

    function CloseAnimation(~, ~)
        if ishandle(fig)
            if isappdata(fig, "AnimationTimer")
                tmr = getappdata(fig, "AnimationTimer");
                SafeStopAndDeleteTimer(tmr);
            end
            delete(fig);
        end
    end

    % =====================================================================
    % Atualizacao grafica
    % =====================================================================

    function UpdateFrame(k)
        if ~ishandle(fig)
            return;
        end

        k = max(1, min(k, N));

        data = get(fig, "UserData");
        handles = data.handles;

        pos = position(:, k);
        eta = attitude(:, k);
        vel = velocity(:, k);

        R = RotationMatrixZYX(eta(1), eta(2), eta(3));

        p1 = pos + R*[ armVisualLength; 0; 0];
        p2 = pos + R*[-armVisualLength; 0; 0];
        p3 = pos + R*[0;  armVisualLength; 0];
        p4 = pos + R*[0; -armVisualLength; 0];

        pHeading = pos + R*[1.35*armVisualLength; 0; 0];

        set(handles.hArmX, ...
            "XData", [p1(1) p2(1)], ...
            "YData", [p1(2) p2(2)], ...
            "ZData", [p1(3) p2(3)]);

        set(handles.hArmY, ...
            "XData", [p3(1) p4(1)], ...
            "YData", [p3(2) p4(2)], ...
            "ZData", [p3(3) p4(3)]);

        motorPoints = [p1 p2 p3 p4];

        set(handles.hMotors, ...
            "XData", motorPoints(1, :), ...
            "YData", motorPoints(2, :), ...
            "ZData", motorPoints(3, :));

        set(handles.hCenter, ...
            "XData", pos(1), ...
            "YData", pos(2), ...
            "ZData", pos(3));

        set(handles.hHeading, ...
            "XData", [pos(1) pHeading(1)], ...
            "YData", [pos(2) pHeading(2)], ...
            "ZData", [pos(3) pHeading(3)]);

        selectedLapNow = data.selectedLap;

        for lap = 1:numLaps
            if selectedLapNow ~= 0 && selectedLapNow ~= lap
                set(handles.hActual(lap), ...
                    "XData", nan, ...
                    "YData", nan, ...
                    "ZData", nan);
                continue;
            end

            lapMask = lapIndex(1:k) == lap;

            if any(lapMask)
                idxLap = find(lapMask);

                set(handles.hActual(lap), ...
                    "XData", position(1, idxLap), ...
                    "YData", position(2, idxLap), ...
                    "ZData", position(3, idxLap));
            end
        end

        currentLap = lapIndex(k);
        currentSegment = segmentIndex(k);

        omegaNow = motorOmega(:, k);

        formatSpec = [ ...
            't = %.2f s\n' ...
            'Amostra = %d / %d\n\n' ...
            'Volta = %d / %d\n' ...
            'Segmento = %d / %d\n\n' ...
            'Posicao [m]\n' ...
            '  x = %+8.3f\n' ...
            '  y = %+8.3f\n' ...
            '  z = %+8.3f\n\n' ...
            'Velocidade [m/s]\n' ...
            '  vx = %+8.3f\n' ...
            '  vy = %+8.3f\n' ...
            '  vz = %+8.3f\n\n' ...
            'Atitude [graus]\n' ...
            '  phi   = %+8.3f\n' ...
            '  theta = %+8.3f\n' ...
            '  psi   = %+8.3f\n\n' ...
            'Motores [rad/s]\n' ...
            '  w1 = %+8.1f\n' ...
            '  w2 = %+8.1f\n' ...
            '  w3 = %+8.1f\n' ...
            '  w4 = %+8.1f\n\n' ...
            'Massa = %.4f kg'];

        infoText = sprintf( ...
            formatSpec, ...
            t(k), ...
            k, N, ...
            currentLap, numLaps, ...
            currentSegment, numSegmentsPerLap, ...
            pos(1), pos(2), pos(3), ...
            vel(1), vel(2), vel(3), ...
            rad2deg(eta(1)), rad2deg(eta(2)), rad2deg(eta(3)), ...
            omegaNow(1), omegaNow(2), omegaNow(3), omegaNow(4), ...
            massHistory(k));

        set(handles.infoBox, "String", infoText);

        drawnow limitrate;
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

function armVisualLength = GetVisualArmLength(simData, position, refPosition)
    defaultArmLength = 0.5;

    if isfield(simData, "config") && ...
       isfield(simData.config, "quad") && ...
       isfield(simData.config.quad, "armLength")
        physicalArmLength = simData.config.quad.armLength;
    else
        physicalArmLength = defaultArmLength;
    end

    allPoints = [position, refPosition];
    validColumns = all(isfinite(allPoints), 1);

    if ~any(validColumns)
        armVisualLength = physicalArmLength;
        return;
    end

    allPoints = allPoints(:, validColumns);

    minValue = min(allPoints, [], 2);
    maxValue = max(allPoints, [], 2);

    trajectoryRange = max(maxValue - minValue);

    if trajectoryRange <= 0
        trajectoryRange = 1;
    end

    visualMinimum = 0.035*trajectoryRange;

    armVisualLength = max(physicalArmLength, visualMinimum);
end

function R = RotationMatrixZYX(phi, theta, psi)
    cphi = cos(phi);
    sphi = sin(phi);

    ctheta = cos(theta);
    stheta = sin(theta);

    cpsi = cos(psi);
    spsi = sin(psi);

    Rz = [
        cpsi, -spsi, 0;
        spsi,  cpsi, 0;
        0,     0,    1
    ];

    Ry = [
        ctheta, 0, stheta;
        0,      1, 0;
       -stheta, 0, ctheta
    ];

    Rx = [
        1, 0,     0;
        0, cphi, -sphi;
        0, sphi,  cphi
    ];

    R = Rz*Ry*Rx;
end

function SafeStopTimer(tmr)
    if isempty(tmr)
        return;
    end

    if isvalid(tmr) && strcmp(tmr.Running, "on")
        stop(tmr);
    end
end

function SafeStopAndDeleteTimer(tmr)
    if isempty(tmr)
        return;
    end

    if isvalid(tmr)
        if strcmp(tmr.Running, "on")
            stop(tmr);
        end
        delete(tmr);
    end
end