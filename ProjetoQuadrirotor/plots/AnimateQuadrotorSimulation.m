function AnimateQuadrotorSimulation(simData, plotConfig)

    if nargin < 2 || isempty(plotConfig)
        plotConfig = struct();
    end

    if ~isfield(simData, "state") || ~isfield(simData.state, "x")
        warning("AnimateQuadrotorSimulation: simData.state.x nao encontrado.");
        return;
    end

    state = simData.state.x;
    N = size(state, 2);

    if N < 1
        warning("AnimateQuadrotorSimulation: simulacao vazia.");
        return;
    end

    t = GetTimeVector(simData, N);

    position = GetStateBlock(state, 1, 3, N);
    velocity = GetStateBlock(state, 4, 6, N);
    attitude = GetStateBlock(state, 7, 9, N);

    reference = GetReferencePosition(simData, N);
    motorOmega = GetMotorOmega(simData, N);

    lapIndex = GetConditionVector(simData, "lap", N, ones(1, N));
    segmentIndex = GetConditionVector(simData, "segment", N, ones(1, N));
    massVector = GetConditionVector(simData, "mass", N, nan(1, N));

    lapIndex = round(lapIndex);
    segmentIndex = round(segmentIndex);

    lapIndex(lapIndex < 1 | ~isfinite(lapIndex)) = 1;
    segmentIndex(segmentIndex < 1 | ~isfinite(segmentIndex)) = 1;

    numLaps = max(lapIndex);

    if isempty(numLaps) || ~isfinite(numLaps) || numLaps < 1
        numLaps = 1;
    end

    numLaps = round(numLaps);

    animationStep = GetPlotConfigValue(plotConfig, "animationStep", "AnimationStep", 20);
    animationPeriod = GetPlotConfigValue(plotConfig, "animationPeriod", "AnimationPeriod", 0.04);

    animationStep = max(1, round(animationStep));
    animationPeriod = max(0.01, animationPeriod);

    droneGeom = GetFixedDroneGeometry();

    currentSampleIndex = 1;
    isPaused = false;
    isStopped = false;

    darkBg = [0.10 0.10 0.10];
    darkPanel = [0.14 0.14 0.14];
    darkAxes = [0.07 0.07 0.07];
    textColor = [0.95 0.95 0.95];
    gridColor = [0.30 0.30 0.30];

    colors = lines(max(numLaps, 1));

    existingFig = findall(0, "Type", "figure", "Tag", "QuadrotorAnimationFigure");

    for iFig = 1:numel(existingFig)
        try
            if isappdata(existingFig(iFig), "AnimationTimer")
                oldTimer = getappdata(existingFig(iFig), "AnimationTimer");
                SafeStopAndDeleteTimer(oldTimer);
            end

            if ishandle(existingFig(iFig))
                set(existingFig(iFig), "CloseRequestFcn", "closereq");
                delete(existingFig(iFig));
            end
        catch
        end
    end

    fig = figure( ...
        "Name", "Animacao da simulacao do quadrotor", ...
        "NumberTitle", "off", ...
        "Color", darkBg, ...
        "Tag", "QuadrotorAnimationFigure", ...
        "CloseRequestFcn", @CloseAnimationFigure);

    ax = axes( ...
        "Parent", fig, ...
        "Position", [0.055 0.105 0.62 0.80], ...
        "Color", darkAxes, ...
        "XColor", textColor, ...
        "YColor", textColor, ...
        "ZColor", textColor, ...
        "GridColor", gridColor, ...
        "MinorGridColor", gridColor);

    hold(ax, "on");
    grid(ax, "on");
    box(ax, "on");
    axis(ax, "equal");
    view(ax, 45, 25);

    xlabel(ax, "x [m]", "Color", textColor);
    ylabel(ax, "y [m]", "Color", textColor);
    zlabel(ax, "z [m]", "Color", textColor);

    title(ax, "Animacao 3D do quadrotor", ...
        "Color", textColor, ...
        "FontWeight", "bold");

    ConfigureGlobalAxisLimits();

    hRef = gobjects(numLaps, 1);
    hReal = gobjects(numLaps, 1);

    for lap = 1:numLaps

        lapMask = lapIndex == lap;

        hRef(lap) = plot3(ax, ...
            reference(1, lapMask), ...
            reference(2, lapMask), ...
            reference(3, lapMask), ...
            "--", ...
            "Color", colors(lap, :), ...
            "LineWidth", 1.2, ...
            "DisplayName", sprintf("Ref. volta %d", lap));

        hReal(lap) = plot3(ax, ...
            position(1, lapMask), ...
            position(2, lapMask), ...
            position(3, lapMask), ...
            "-", ...
            "Color", colors(lap, :), ...
            "LineWidth", 1.6, ...
            "DisplayName", sprintf("Real volta %d", lap));
    end

    lgd = legend(ax, "Location", "northeastoutside");
    lgd.TextColor = textColor;
    lgd.Color = darkPanel;
    lgd.EdgeColor = gridColor;

    droneGraphics = CreateDroneGraphics(ax);
    UpdateDroneGraphics(droneGraphics, droneGeom, position(:, 1), attitude(:, 1));

    infoPanel = uipanel( ...
        "Parent", fig, ...
        "Units", "normalized", ...
        "Position", [0.705 0.34 0.275 0.53], ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "BorderType", "none");
    
    txtInfo = uicontrol( ...
        "Parent", infoPanel, ...
        "Style", "text", ...
        "Units", "normalized", ...
        "Position", [0.03 0.02 0.94 0.96], ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "HorizontalAlignment", "left", ...
        "FontName", "Consolas", ...
        "FontSize", 9, ...
        "String", "");

    statusPanel = uipanel( ...
        "Parent", fig, ...
        "Units", "normalized", ...
        "Position", [0.705 0.285 0.275 0.065], ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "BorderType", "none");

    txtStatus = uicontrol( ...
        "Parent", statusPanel, ...
        "Style", "text", ...
        "Units", "normalized", ...
        "Position", [0.04 0.08 0.92 0.84], ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "HorizontalAlignment", "left", ...
        "FontWeight", "bold", ...
        "String", "Status: rodando");

    uicontrol( ...
        "Parent", fig, ...
        "Style", "text", ...
        "Units", "normalized", ...
        "Position", [0.705 0.235 0.105 0.035], ...
        "BackgroundColor", darkPanel, ...
        "ForegroundColor", textColor, ...
        "HorizontalAlignment", "left", ...
        "FontWeight", "bold", ...
        "String", "Mostrar:");

    lapOptions = cell(numLaps + 1, 1);
    lapOptions{1} = "Todas";

    for lap = 1:numLaps
        lapOptions{lap + 1} = sprintf("Volta %d", lap);
    end

    popupLap = uicontrol( ...
        "Parent", fig, ...
        "Style", "popupmenu", ...
        "Units", "normalized", ...
        "Position", [0.815 0.235 0.165 0.038], ...
        "String", lapOptions, ...
        "BackgroundColor", [0.18 0.18 0.18], ...
        "ForegroundColor", textColor, ...
        "Callback", @SelectLap);

    btnPause = uicontrol( ...
        "Parent", fig, ...
        "Style", "pushbutton", ...
        "Units", "normalized", ...
        "Position", [0.705 0.155 0.130 0.052], ...
        "String", "Pausar", ...
        "FontWeight", "bold", ...
        "BackgroundColor", [0.16 0.16 0.16], ...
        "ForegroundColor", textColor, ...
        "Callback", @PauseAnimation);

    btnStop = uicontrol( ...
        "Parent", fig, ...
        "Style", "pushbutton", ...
        "Units", "normalized", ...
        "Position", [0.850 0.155 0.130 0.052], ...
        "String", "Parar", ...
        "FontWeight", "bold", ...
        "BackgroundColor", [0.16 0.16 0.16], ...
        "ForegroundColor", textColor, ...
        "Callback", @StopAnimation);

    btnRestart = uicontrol( ...
        "Parent", fig, ...
        "Style", "pushbutton", ...
        "Units", "normalized", ...
        "Position", [0.705 0.085 0.275 0.052], ...
        "String", "Reiniciar", ...
        "FontWeight", "bold", ...
        "BackgroundColor", [0.16 0.16 0.16], ...
        "ForegroundColor", textColor, ...
        "Callback", @RestartAnimation);

    UpdateFrame(1);

    animationTimer = timer( ...
        "ExecutionMode", "fixedSpacing", ...
        "Period", animationPeriod, ...
        "BusyMode", "drop", ...
        "TimerFcn", @TimerStep);

    setappdata(fig, "AnimationTimer", animationTimer);
    start(animationTimer);
    drawnow;

    function TimerStep(~, ~)

        try
            if ~ishandle(fig)
                SafeStopAndDeleteTimer(animationTimer);
                return;
            end

            if isPaused || isStopped
                return;
            end

            currentSampleIndex = currentSampleIndex + animationStep;

            if currentSampleIndex >= N
                currentSampleIndex = N;
                UpdateFrame(currentSampleIndex);
                set(txtStatus, "String", "Status: finalizada");
                SafeStopTimer(animationTimer);
                return;
            end

            UpdateFrame(currentSampleIndex);

        catch ME
            try
                set(txtStatus, "String", "Status: erro na animacao");
            catch
            end

            disp("Erro no timer da animacao:");
            disp(getReport(ME, "extended"));

            SafeStopTimer(animationTimer);
        end
    end

    function UpdateFrame(k)

        k = max(1, min(N, round(k)));
        currentSampleIndex = k;

        positionNow = position(:, k);
        velocityNow = velocity(:, k);
        attitudeNow = attitude(:, k);

        UpdateDroneGraphics(droneGraphics, droneGeom, positionNow, attitudeNow);

        infoLines = BuildInfoText(k, positionNow, velocityNow, attitudeNow);
        set(txtInfo, "String", infoLines);

        drawnow limitrate;
    end

    function infoLines = BuildInfoText(k, positionNow, velocityNow, attitudeNow)

        lapNow = lapIndex(k);
        segmentNow = segmentIndex(k);

        if isfinite(massVector(k))
            massLine = sprintf("massa = %+8.4f kg", massVector(k));
        else
            massLine = "massa =    n/a";
        end

        infoLines = {
            sprintf("t = %8.2f s", t(k));
            sprintf("Amostra = %d / %d", k, N);
            "";
            sprintf("Volta = %d / %d", lapNow, numLaps);
            sprintf("Segmento = %d", segmentNow);
            "";
            "Posicao [m]";
            sprintf("   x = %+8.3f", positionNow(1));
            sprintf("   y = %+8.3f", positionNow(2));
            sprintf("   z = %+8.3f", positionNow(3));
            "";
            "Velocidade [m/s]";
            sprintf("   vx = %+8.3f", velocityNow(1));
            sprintf("   vy = %+8.3f", velocityNow(2));
            sprintf("   vz = %+8.3f", velocityNow(3));
            "";
            "Atitude [graus]";
            sprintf("   phi   = %+8.3f", rad2deg(attitudeNow(1)));
            sprintf("   theta = %+8.3f", rad2deg(attitudeNow(2)));
            sprintf("   psi   = %+8.3f", rad2deg(attitudeNow(3)));
            "";
            "Motor [rad/s]";
            sprintf("   w1 = %+8.1f", motorOmega(1,k));
            sprintf("   w2 = %+8.1f", motorOmega(2,k));
            sprintf("   w3 = %+8.1f", motorOmega(3,k));
            sprintf("   w4 = %+8.1f", motorOmega(4,k));
            "";
            char(massLine)
        };
    end

    function SelectLap(~, ~)

        selectedLap = get(popupLap, "Value") - 1;

        for iLap = 1:numLaps
            if selectedLap == 0 || selectedLap == iLap
                set(hRef(iLap), "Visible", "on");
                set(hReal(iLap), "Visible", "on");
            else
                set(hRef(iLap), "Visible", "off");
                set(hReal(iLap), "Visible", "off");
            end
        end

        if selectedLap > 0
            firstIndex = find(lapIndex == selectedLap, 1, "first");

            if ~isempty(firstIndex)
                currentSampleIndex = firstIndex;
                UpdateFrame(currentSampleIndex);
            end
        end

        ConfigureGlobalAxisLimits();
    end

    function ConfigureGlobalAxisLimits()

        allPoints = [position, reference];
        validColumns = all(isfinite(allPoints), 1);
        allPoints = allPoints(:, validColumns);

        if isempty(allPoints)
            xlim(ax, [-1 1]);
            ylim(ax, [-1 1]);
            zlim(ax, [-1 1]);
            return;
        end

        minValue = min(allPoints, [], 2);
        maxValue = max(allPoints, [], 2);

        center = 0.5*(minValue + maxValue);
        rangeValue = max(maxValue - minValue);

        if ~isfinite(rangeValue) || rangeValue <= 0
            rangeValue = 1;
        end

        margin = 0.20*rangeValue;
        halfRange = 0.5*rangeValue + margin;

        xlim(ax, [center(1)-halfRange, center(1)+halfRange]);
        ylim(ax, [center(2)-halfRange, center(2)+halfRange]);
        zlim(ax, [center(3)-halfRange, center(3)+halfRange]);

        view(ax, 45, 25);
    end

    function PauseAnimation(~, ~)

        if isStopped
            return;
        end

        isPaused = ~isPaused;

        if isPaused
            set(btnPause, "String", "Continuar");
            set(txtStatus, "String", "Status: pausada");
        else
            set(btnPause, "String", "Pausar");
            set(txtStatus, "String", "Status: rodando");
        end
    end

    function StopAnimation(~, ~)

        isStopped = true;
        isPaused = false;

        SafeStopTimer(animationTimer);

        set(btnPause, "String", "Pausar");
        set(txtStatus, "String", "Status: parada");
    end

    function RestartAnimation(~, ~)

        isStopped = false;
        isPaused = false;
        currentSampleIndex = 1;

        set(btnPause, "String", "Pausar");
        set(txtStatus, "String", "Status: rodando");

        UpdateFrame(currentSampleIndex);

        if strcmp(animationTimer.Running, "off")
            start(animationTimer);
        end
    end

    function CloseAnimationFigure(~, ~)

        try
            if isappdata(fig, "AnimationTimer")
                timerToDelete = getappdata(fig, "AnimationTimer");
                SafeStopAndDeleteTimer(timerToDelete);
            end
        catch
        end

        try
            delete(fig);
        catch
        end
    end
end

function value = GetPlotConfigValue(plotConfig, lowerName, upperName, defaultValue)

    value = defaultValue;

    if isstruct(plotConfig)
        if isfield(plotConfig, lowerName)
            value = plotConfig.(lowerName);
            return;
        end

        if isfield(plotConfig, upperName)
            value = plotConfig.(upperName);
            return;
        end
    end
end

function t = GetTimeVector(simData, N)

    if isfield(simData, "t")
        t = simData.t(:).';

        if numel(t) == N
            return;
        end
    end

    t = 0:(N-1);
end

function block = GetStateBlock(state, firstIndex, lastIndex, N)

    block = zeros(lastIndex - firstIndex + 1, N);

    if size(state, 1) >= lastIndex
        block = state(firstIndex:lastIndex, :);
    end
end

function reference = GetReferencePosition(simData, N)

    reference = nan(3, N);

    if isfield(simData, "ref") && isfield(simData.ref, "r")
        ref = simData.ref.r;

        if size(ref, 1) >= 3
            nRef = min(N, size(ref, 2));
            reference(:, 1:nRef) = ref(1:3, 1:nRef);
        end
    end
end

function motorOmega = GetMotorOmega(simData, N)

    motorOmega = nan(4, N);

    if isfield(simData, "cmd") && isfield(simData.cmd, "motorOmega")
        omega = simData.cmd.motorOmega;

        if size(omega, 1) >= 4
            nOmega = min(N, size(omega, 2));
            motorOmega(:, 1:nOmega) = omega(1:4, 1:nOmega);
        end
    end
end

function value = GetConditionVector(simData, fieldName, N, defaultValue)

    value = defaultValue;

    if ~isfield(simData, "condition")
        return;
    end

    if ~isfield(simData.condition, fieldName)
        return;
    end

    raw = simData.condition.(fieldName);

    if isempty(raw)
        return;
    end

    raw = raw(:).';

    if numel(raw) == N
        value = raw;
        return;
    end

    nRaw = min(N, numel(raw));
    value(1:nRaw) = raw(1:nRaw);
end

function droneGeom = GetFixedDroneGeometry()

    droneGeom.armLength = 0.50;
    droneGeom.bodyLength = 0.28;
    droneGeom.bodyWidth = 0.15;
    droneGeom.rotorRadius = 0.10;
end

function droneGraphics = CreateDroneGraphics(ax)

    droneGraphics.armX = plot3(ax, nan, nan, nan, ...
        "-", ...
        "Color", [1 1 1], ...
        "LineWidth", 2.0, ...
        "HandleVisibility", "off");

    droneGraphics.armY = plot3(ax, nan, nan, nan, ...
        "-", ...
        "Color", [1 1 1], ...
        "LineWidth", 2.0, ...
        "HandleVisibility", "off");

    droneGraphics.body = plot3(ax, nan, nan, nan, ...
        "-", ...
        "Color", [0.85 0.85 0.85], ...
        "LineWidth", 2.0, ...
        "HandleVisibility", "off");

    droneGraphics.motor = gobjects(4, 1);
    droneGraphics.rotor = gobjects(4, 1);

    for i = 1:4
        droneGraphics.motor(i) = plot3(ax, nan, nan, nan, ...
            "o", ...
            "MarkerSize", 4, ...
            "MarkerFaceColor", [1 1 1], ...
            "MarkerEdgeColor", [1 1 1], ...
            "HandleVisibility", "off");

        droneGraphics.rotor(i) = plot3(ax, nan, nan, nan, ...
            "-", ...
            "Color", [0.90 0.90 0.90], ...
            "LineWidth", 1.0, ...
            "HandleVisibility", "off");
    end

    droneGraphics.frontMarker = plot3(ax, nan, nan, nan, ...
        "o", ...
        "MarkerSize", 5, ...
        "MarkerFaceColor", [1.0 0.35 0.20], ...
        "MarkerEdgeColor", [1.0 0.35 0.20], ...
        "HandleVisibility", "off");
end

function UpdateDroneGraphics(droneGraphics, droneGeom, positionNow, attitudeNow)

    phi = attitudeNow(1);
    theta = attitudeNow(2);
    psi = attitudeNow(3);

    R = RotationZYX(phi, theta, psi);

    L = droneGeom.armLength;
    BL = droneGeom.bodyLength;
    BW = droneGeom.bodyWidth;
    RR = droneGeom.rotorRadius;

    positionNow = positionNow(:);

    armXLocal = [
        -L, L;
         0, 0;
         0, 0
    ];

    armYLocal = [
         0, 0;
        -L, L;
         0, 0
    ];

    armXWorld = positionNow + R*armXLocal;
    armYWorld = positionNow + R*armYLocal;

    set(droneGraphics.armX, ...
        "XData", armXWorld(1, :), ...
        "YData", armXWorld(2, :), ...
        "ZData", armXWorld(3, :));

    set(droneGraphics.armY, ...
        "XData", armYWorld(1, :), ...
        "YData", armYWorld(2, :), ...
        "ZData", armYWorld(3, :));

    bodyLocal = [
        -BL/2,  BL/2,  BL/2, -BL/2, -BL/2;
        -BW/2, -BW/2,  BW/2,  BW/2, -BW/2;
          0,     0,     0,     0,     0
    ];

    bodyWorld = positionNow + R*bodyLocal;

    set(droneGraphics.body, ...
        "XData", bodyWorld(1, :), ...
        "YData", bodyWorld(2, :), ...
        "ZData", bodyWorld(3, :));

    motorLocal = [
         L, -L,  0,  0;
         0,  0,  L, -L;
         0,  0,  0,  0
    ];

    motorWorld = positionNow + R*motorLocal;

    for i = 1:4
        set(droneGraphics.motor(i), ...
            "XData", motorWorld(1, i), ...
            "YData", motorWorld(2, i), ...
            "ZData", motorWorld(3, i));

        rotorLocal = RotorCircleLocal(motorLocal(:, i), RR);
        rotorWorld = positionNow + R*rotorLocal;

        set(droneGraphics.rotor(i), ...
            "XData", rotorWorld(1, :), ...
            "YData", rotorWorld(2, :), ...
            "ZData", rotorWorld(3, :));
    end

    frontLocal = [L; 0; 0];
    frontWorld = positionNow + R*frontLocal;

    set(droneGraphics.frontMarker, ...
        "XData", frontWorld(1), ...
        "YData", frontWorld(2), ...
        "ZData", frontWorld(3));
end

function circleLocal = RotorCircleLocal(centerLocal, rotorRadius)

    beta = linspace(0, 2*pi, 24);

    circleLocal = [
        centerLocal(1) + rotorRadius*cos(beta);
        centerLocal(2) + rotorRadius*sin(beta);
        centerLocal(3) + zeros(size(beta))
    ];
end

function R = RotationZYX(phi, theta, psi)

    cphi = cos(phi);
    sphi = sin(phi);

    ctheta = cos(theta);
    stheta = sin(theta);

    cpsi = cos(psi);
    spsi = sin(psi);

    Rx = [
        1,    0,     0;
        0, cphi, -sphi;
        0, sphi,  cphi
    ];

    Ry = [
         ctheta, 0, stheta;
              0, 1,      0;
        -stheta, 0, ctheta
    ];

    Rz = [
        cpsi, -spsi, 0;
        spsi,  cpsi, 0;
           0,     0, 1
    ];

    R = Rz*Ry*Rx;
end

function SafeStopTimer(timerObj)

    try
        if isa(timerObj, "timer") && isvalid(timerObj)
            if strcmp(timerObj.Running, "on")
                stop(timerObj);
            end
        end
    catch
    end
end

function SafeStopAndDeleteTimer(timerObj)

    try
        if isa(timerObj, "timer") && isvalid(timerObj)
            if strcmp(timerObj.Running, "on")
                stop(timerObj);
            end

            delete(timerObj);
        end
    catch
    end
end