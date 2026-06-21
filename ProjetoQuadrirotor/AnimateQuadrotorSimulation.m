function anim = AnimateQuadrotorSimulation(sim, varargin)
% AnimateQuadrotorSimulation
% -------------------------------------------------------------------------
% Anima o movimento 3D do quadrirotor a partir da struct de simulacao.
%
% Uso recomendado:
%
%   AnimateQuadrotorSimulation(sim, ...
%       "Step", 10, ...
%       "DelayMode", "fixed", ...
%       "FixedDelay", 0.03, ...
%       "ColorByLap", true, ...
%       "Theme", "dark");
%
% Campos esperados em sim:
%   sim.t              -> vetor de tempo
%   sim.state          -> matriz 12xN ou Nx12
%   sim.r_des          -> referencia de posicao 3xN ou Nx3, opcional
%   sim.lapIndex       -> indice da volta, opcional
%   sim.motorOmega     -> velocidades angulares dos motores 4xN, opcional
%   sim.omegaSquared   -> quadrado das velocidades dos motores 4xN, opcional
%   sim.quad.armLength -> comprimento do braco, opcional
% -------------------------------------------------------------------------

    %% ------------------------- Entradas ---------------------------------

    if nargin < 1 || isempty(sim)
        error('Informe a struct sim gerada pela simulacao.');
    end

    p = inputParser;

    addParameter(p, 'SaveGif', false, @(x) islogical(x) && isscalar(x));
    addParameter(p, 'GifFile', 'animacao_quadrirotor.gif', @(x) ischar(x) || isstring(x));
    addParameter(p, 'FrameRate', 30, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'PlaybackSpeed', 1, @(x) isnumeric(x) && isscalar(x) && x > 0);

    addParameter(p, 'DelayMode', 'fixed', ...
        @(x) any(strcmpi(string(x), ["simulation", "fixed"])));

    addParameter(p, 'FixedDelay', 0.03, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, 'MinDelay', 0.01, @(x) isnumeric(x) && isscalar(x) && x >= 0);

    addParameter(p, 'Step', 10, ...
        @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x >= 1));

    addParameter(p, 'ShowReference', true, @(x) islogical(x) && isscalar(x));
    addParameter(p, 'ShowTrail', true, @(x) islogical(x) && isscalar(x));
    addParameter(p, 'ColorByLap', true, @(x) islogical(x) && isscalar(x));

    addParameter(p, 'UsePhysicalSize', false, @(x) islogical(x) && isscalar(x));
    addParameter(p, 'ArmLength', [], ...
        @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));

    addParameter(p, 'View', [35 25], @(x) isnumeric(x) && numel(x) == 2);
    addParameter(p, 'ExternalFigure', true, @(x) islogical(x) && isscalar(x));
    addParameter(p, 'ShowLegend', true, @(x) islogical(x) && isscalar(x));

    addParameter(p, 'Theme', 'dark', ...
        @(x) any(strcmpi(string(x), ["dark", "light"])));

    parse(p, varargin{:});

    opt = p.Results;
    opt.GifFile = char(opt.GifFile);
    opt.DelayMode = lower(string(opt.DelayMode));
    opt.Theme = lower(string(opt.Theme));

    if isempty(opt.Step)
        manualStep = [];
    else
        manualStep = max(1, round(opt.Step));
    end

    %% ---------------------- Dados da simulacao --------------------------

    if ~isfield(sim, 't') || isempty(sim.t)
        error('A struct sim precisa conter o campo sim.t.');
    end

    if ~isfield(sim, 'state') || isempty(sim.state)
        error('A struct sim precisa conter o campo sim.state.');
    end

    t = sim.t(:).';
    state = sim.state;

    if size(state, 1) == 12
        % Formato esperado: 12xN.
    elseif size(state, 2) == 12
        state = state.';
    else
        error('sim.state deve ter dimensao 12xN ou Nx12.');
    end

    N = min(numel(t), size(state, 2));

    t = t(1:N);
    state = state(:, 1:N);

    pos = state(1:3, :);
    att = state(7:9, :);

    motorOmega = LocalGetMotorOmega(sim, N);

    hasReference = isfield(sim, 'r_des') && ~isempty(sim.r_des);

    if hasReference
        rDes = LocalEnsure3xN(sim.r_des, 'sim.r_des');

        if size(rDes, 2) < N
            rDes = [rDes, repmat(rDes(:,end), 1, N - size(rDes,2))];
        end

        rDes = rDes(:, 1:N);
    else
        rDes = [];
    end

    showReference = opt.ShowReference && hasReference;

    %% ---------------------- Cores e tema --------------------------------

    if opt.Theme == "dark"
        figColor    = [0.06 0.06 0.06];
        axColor     = [0.08 0.08 0.08];
        gridColor   = [0.45 0.45 0.45];
        textColor   = [0.92 0.92 0.92];
        legendColor = [0.10 0.10 0.10];
    else
        figColor    = [1.00 1.00 1.00];
        axColor     = [1.00 1.00 1.00];
        gridColor   = [0.30 0.30 0.30];
        textColor   = [0.00 0.00 0.00];
        legendColor = [1.00 1.00 1.00];
    end

    %% ---------------------- Deteccao das voltas -------------------------

    if opt.ColorByLap
        [lapIndex, uniqueLaps, nLaps] = LocalGetLapIndex(sim, pos, N);
        lapColors = lines(max(nLaps, 1));
    else
        lapIndex = ones(1, N);
        uniqueLaps = 1;
        nLaps = 1;
        lapColors = [0.0000 0.4470 0.7410];
    end

    %% ---------------------- Amostragem dos frames -----------------------

    if N < 2
        error('A simulacao precisa ter pelo menos duas amostras.');
    end

    TsMedio = median(diff(t));

    if TsMedio <= 0 || ~isfinite(TsMedio)
        TsMedio = 0.01;
    end

    if isempty(manualStep)
        step = max(1, round(opt.PlaybackSpeed/(opt.FrameRate*TsMedio)));
    else
        step = manualStep;
    end

    frameIndex = 1:step:N;

    if frameIndex(end) ~= N
        frameIndex = [frameIndex, N];
    end

    fprintf('\n--- Animacao do quadrirotor ---\n');
    fprintf('Numero de amostras da simulacao: %d\n', N);
    fprintf('Passo entre frames: %d\n', step);
    fprintf('Numero de frames da animacao: %d\n', numel(frameIndex));
    fprintf('Numero de voltas detectadas: %d\n', nLaps);

    %% ---------------------- Dimensoes visuais do drone ------------------

    armPhysical = 0.25;

    if isfield(sim, 'quad') && isstruct(sim.quad) && isfield(sim.quad, 'armLength')
        armPhysical = sim.quad.armLength;
    end

    if ~isempty(opt.ArmLength)
        armPhysical = opt.ArmLength;
    end

    allXYZ = pos;

    if showReference
        allXYZ = [allXYZ, rDes];
    end

    validXYZ = all(isfinite(allXYZ), 1);

    if any(validXYZ)
        allXYZvalid = allXYZ(:, validXYZ);
    else
        allXYZvalid = zeros(3,1);
    end

    xyzMin = min(allXYZvalid, [], 2);
    xyzMax = max(allXYZvalid, [], 2);

    xyzRange = xyzMax - xyzMin;
    sceneSpan = max(xyzRange);

    if sceneSpan <= 0 || ~isfinite(sceneSpan)
        sceneSpan = 1;
    end

    if opt.UsePhysicalSize
        L = armPhysical;
    else
        L = max(armPhysical, 0.04*sceneSpan);
    end

    propRadius = 0.28*L;
    zBodyAxisLength = 0.55*L;

    %% ---------------------- Figura e eixos ------------------------------

    fig = figure( ...
        'Name', 'Animacao do Quadrirotor', ...
        'Color', figColor, ...
        'WindowStyle', 'normal', ...
        'Visible', 'on', ...
        'Position', [100 100 1100 750]);

    set(fig, 'InvertHardcopy', 'off');

    try
        set(fig, 'Renderer', 'opengl');
    catch
    end

    if opt.ExternalFigure
        try
            movegui(fig, 'center');
        catch
        end
    end

    ax = axes('Parent', fig);

    hold(ax, 'on');
    grid(ax, 'on');
    box(ax, 'on');

    axis(ax, 'equal');
    axis(ax, 'vis3d');

    view(ax, opt.View(1), opt.View(2));

    set(ax, ...
        'Color', axColor, ...
        'XColor', textColor, ...
        'YColor', textColor, ...
        'ZColor', textColor, ...
        'GridColor', gridColor, ...
        'MinorGridColor', gridColor, ...
        'LineWidth', 1.0);

    xlabel(ax, 'x [m]', 'Color', textColor);
    ylabel(ax, 'y [m]', 'Color', textColor);
    zlabel(ax, 'z [m]', 'Color', textColor);
    title(ax, 'Movimento 3D do quadrirotor', 'Color', textColor);

    %% ---------------------- Trajetorias fixas ---------------------------

    if opt.ColorByLap

        for iLap = 1:nLaps

            lapNumber = uniqueLaps(iLap);
            idxLap = find(lapIndex == lapNumber);

            if isempty(idxLap)
                continue;
            end

            plot3(ax, pos(1,idxLap), pos(2,idxLap), pos(3,idxLap), ':', ...
                'LineWidth', 1.2, ...
                'Color', lapColors(iLap,:), ...
                'DisplayName', sprintf('traj. real volta %d', lapNumber));

            if showReference
                idxRef = idxLap(idxLap <= size(rDes,2));

                plot3(ax, rDes(1,idxRef), rDes(2,idxRef), rDes(3,idxRef), '--', ...
                    'LineWidth', 1.2, ...
                    'Color', lapColors(iLap,:), ...
                    'HandleVisibility', 'off');
            end
        end

        if showReference
            plot3(ax, nan, nan, nan, '--', ...
                'LineWidth', 1.2, ...
                'Color', [0.75 0.75 0.75], ...
                'DisplayName', 'referencia');
        end

    else

        plot3(ax, pos(1,:), pos(2,:), pos(3,:), ':', ...
            'LineWidth', 1.0, ...
            'DisplayName', 'trajetoria simulada');

        if showReference
            plot3(ax, rDes(1,:), rDes(2,:), rDes(3,:), '--', ...
                'LineWidth', 1.2, ...
                'DisplayName', 'referencia');
        end
    end

    %% ---------------------- Limites da cena -----------------------------

    margin = max(0.15*sceneSpan, 3*L);

    xlim(ax, [xyzMin(1)-margin, xyzMax(1)+margin]);
    ylim(ax, [xyzMin(2)-margin, xyzMax(2)+margin]);
    zlim(ax, [xyzMin(3)-margin, xyzMax(3)+margin]);

    %% ---------------------- Handles graficos do drone -------------------

    hArmX = plot3(ax, nan, nan, nan, ...
        'LineWidth', 3.0, ...
        'Color', [0.9290 0.6940 0.1250], ...
        'DisplayName', 'braco x_b');

    hArmY = plot3(ax, nan, nan, nan, ...
        'LineWidth', 3.0, ...
        'Color', [0.4940 0.1840 0.5560], ...
        'DisplayName', 'braco y_b');

    hBody = plot3(ax, nan, nan, nan, 'o', ...
        'MarkerSize', 8, ...
        'MarkerFaceColor', [0.4660 0.6740 0.1880], ...
        'MarkerEdgeColor', [0.4660 0.6740 0.1880], ...
        'DisplayName', 'corpo');

    hHeading = plot3(ax, nan, nan, nan, ...
        'LineWidth', 2.0, ...
        'Color', [0.3010 0.7450 0.9330], ...
        'DisplayName', 'frente x_b');

    hZBody = plot3(ax, nan, nan, nan, ...
        'LineWidth', 2.0, ...
        'Color', [0.6350 0.0780 0.1840], ...
        'DisplayName', 'eixo z_b');

    hRotor = gobjects(4,1);

    rotorColors = [
        0.0000 0.4470 0.7410;
        0.8500 0.3250 0.0980;
        0.9290 0.6940 0.1250;
        0.4940 0.1840 0.5560
    ];

    for i = 1:4
        hRotor(i) = plot3(ax, nan, nan, nan, ...
            'LineWidth', 1.3, ...
            'Color', rotorColors(i,:), ...
            'DisplayName', sprintf('rotor %d', i));
    end

    %% ---------------------- Rastro animado ------------------------------

    if opt.ShowTrail
        if opt.ColorByLap

            hTrail = gobjects(nLaps, 1);

            for iLap = 1:nLaps

                lapNumber = uniqueLaps(iLap);

                hTrail(iLap) = plot3(ax, nan, nan, nan, ...
                    'LineWidth', 2.4, ...
                    'Color', lapColors(iLap,:), ...
                    'DisplayName', sprintf('rastro volta %d', lapNumber));
            end

        else
            hTrail = plot3(ax, nan, nan, nan, ...
                'LineWidth', 1.8, ...
                'DisplayName', 'rastro');
        end
    else
        hTrail = gobjects(0);
    end

    hText = text(ax, xyzMin(1), xyzMin(2), xyzMax(3) + 0.45*margin, '', ...
        'FontWeight', 'bold', ...
        'Color', textColor, ...
        'BackgroundColor', axColor, ...
        'EdgeColor', gridColor, ...
        'Margin', 5);

    if opt.ShowLegend
        lgd = legend(ax, 'Location', 'bestoutside');

        set(lgd, ...
            'Color', legendColor, ...
            'TextColor', textColor, ...
            'EdgeColor', gridColor);
    end

    %% ---------------------- Geometria no sistema do corpo ---------------

    motorBody = [
        -L,   0,  L,  0;
         0,  -L,  0,  L;
         0,   0,  0,  0
    ];

    armXBody = [motorBody(:,1), motorBody(:,3)];
    armYBody = [motorBody(:,2), motorBody(:,4)];

    headingBody = [
        [0; 0; 0], ...
        [1.25*L; 0; 0]
    ];

    zBodyAxis = [
        [0; 0; 0], ...
        [0; 0; zBodyAxisLength]
    ];

    circleAngle = linspace(0, 2*pi, 50);

    rotorCircleBody = [
        propRadius*cos(circleAngle);
        propRadius*sin(circleAngle);
        zeros(size(circleAngle))
    ];

    %% ---------------------- Animacao ------------------------------------

    if opt.SaveGif && exist(opt.GifFile, 'file')
        delete(opt.GifFile);
    end

    drawnow;
    pause(0.2);

    for ii = 1:numel(frameIndex)

        if ~isvalid(fig) || ~isvalid(ax)
            break;
        end

        k = frameIndex(ii);

        center = pos(:,k);

        phi   = att(1,k);
        theta = att(2,k);
        psi   = att(3,k);

        R = LocalRotationMatrixZYX(phi, theta, psi);

        armXWorld = center + R*armXBody;
        armYWorld = center + R*armYBody;

        headingWorld = center + R*headingBody;
        zBodyWorld = center + R*zBodyAxis;

        set(hArmX, ...
            'XData', armXWorld(1,:), ...
            'YData', armXWorld(2,:), ...
            'ZData', armXWorld(3,:));

        set(hArmY, ...
            'XData', armYWorld(1,:), ...
            'YData', armYWorld(2,:), ...
            'ZData', armYWorld(3,:));

        set(hHeading, ...
            'XData', headingWorld(1,:), ...
            'YData', headingWorld(2,:), ...
            'ZData', headingWorld(3,:));

        set(hZBody, ...
            'XData', zBodyWorld(1,:), ...
            'YData', zBodyWorld(2,:), ...
            'ZData', zBodyWorld(3,:));

        set(hBody, ...
            'XData', center(1), ...
            'YData', center(2), ...
            'ZData', center(3));

        for motor = 1:4

            circleWorld = center + R*LocalAddColumn(rotorCircleBody, motorBody(:,motor));

            set(hRotor(motor), ...
                'XData', circleWorld(1,:), ...
                'YData', circleWorld(2,:), ...
                'ZData', circleWorld(3,:));
        end

        if opt.ShowTrail
            if opt.ColorByLap

                for iLap = 1:nLaps

                    lapNumber = uniqueLaps(iLap);
                    idxTrail = find(lapIndex(1:k) == lapNumber);

                    set(hTrail(iLap), ...
                        'XData', pos(1,idxTrail), ...
                        'YData', pos(2,idxTrail), ...
                        'ZData', pos(3,idxTrail));
                end

            else

                set(hTrail, ...
                    'XData', pos(1,1:k), ...
                    'YData', pos(2,1:k), ...
                    'ZData', pos(3,1:k));
            end
        end

        if ~isempty(motorOmega)

            w1 = motorOmega(1,k);
            w2 = motorOmega(2,k);
            w3 = motorOmega(3,k);
            w4 = motorOmega(4,k);

            set(hText, 'String', sprintf([ ...
                't = %.2f s\n', ...
                'volta = %d\n', ...
                'phi = %.2f deg, theta = %.2f deg, psi = %.2f deg\n', ...
                'w1 = %.1f rad/s, w2 = %.1f rad/s\n', ...
                'w3 = %.1f rad/s, w4 = %.1f rad/s'], ...
                t(k), round(lapIndex(k)), ...
                rad2deg(phi), rad2deg(theta), rad2deg(psi), ...
                w1, w2, w3, w4));

        else

            set(hText, 'String', sprintf([ ...
                't = %.2f s\n', ...
                'volta = %d\n', ...
                'phi = %.2f deg, theta = %.2f deg, psi = %.2f deg'], ...
                t(k), round(lapIndex(k)), ...
                rad2deg(phi), rad2deg(theta), rad2deg(psi)));
        end

        title(ax, sprintf('Movimento 3D do quadrirotor - t = %.2f s', t(k)), ...
            'Color', textColor);

        drawnow;

        if opt.SaveGif

            frame = getframe(fig);
            [im, map] = rgb2ind(frame2im(frame), 256);

            dtFrame = LocalDelay(ii, frameIndex, t, TsMedio, step, opt);

            if ii == 1
                imwrite(im, map, opt.GifFile, 'gif', ...
                    'LoopCount', inf, ...
                    'DelayTime', dtFrame);
            else
                imwrite(im, map, opt.GifFile, 'gif', ...
                    'WriteMode', 'append', ...
                    'DelayTime', dtFrame);
            end

        else

            if ii < numel(frameIndex)
                pause(LocalDelay(ii, frameIndex, t, TsMedio, step, opt));
            end
        end
    end

    %% ---------------------- Saida ---------------------------------------

    anim = struct();

    anim.figure = fig;
    anim.axes = ax;
    anim.frameIndex = frameIndex;
    anim.visualArmLength = L;
    anim.physicalArmLength = armPhysical;
    anim.step = step;
    anim.numberOfFrames = numel(frameIndex);
    anim.lapIndex = lapIndex;
    anim.uniqueLaps = uniqueLaps;
    anim.numberOfLaps = nLaps;

    if opt.SaveGif
        anim.gifFile = opt.GifFile;
        fprintf('GIF salvo em: %s\n', opt.GifFile);
    end
end

%% =======================================================================
function delay = LocalDelay(ii, frameIndex, t, TsMedio, step, opt)

    if opt.DelayMode == "fixed"

        delay = opt.FixedDelay;

    else

        if ii < numel(frameIndex)
            k = frameIndex(ii);
            kNext = frameIndex(ii+1);
            delay = (t(kNext) - t(k))/opt.PlaybackSpeed;
        else
            delay = TsMedio*step/opt.PlaybackSpeed;
        end
    end

    delay = max(delay, opt.MinDelay);

    if opt.SaveGif
        delay = max(delay, 0.02);
    end
end

%% =======================================================================
function R = LocalRotationMatrixZYX(phi, theta, psi)

    Rz = [
        cos(psi), -sin(psi), 0;
        sin(psi),  cos(psi), 0;
        0,         0,        1
    ];

    Ry = [
         cos(theta), 0, sin(theta);
         0,          1, 0;
        -sin(theta), 0, cos(theta)
    ];

    Rx = [
        1, 0,        0;
        0, cos(phi), -sin(phi);
        0, sin(phi),  cos(phi)
    ];

    R = Rz*Ry*Rx;
end

%% =======================================================================
function data3xN = LocalEnsure3xN(data, name)

    if size(data, 1) == 3
        data3xN = data;
    elseif size(data, 2) == 3
        data3xN = data.';
    else
        error('%s deve ter dimensao 3xN ou Nx3.', name);
    end
end

%% =======================================================================
function out = LocalAddColumn(A, b)

    out = bsxfun(@plus, A, b);
end

%% =======================================================================
function [lapIndex, uniqueLaps, nLaps] = LocalGetLapIndex(sim, pos, N)

    if isfield(sim, 'lapIndex') && ~isempty(sim.lapIndex)

        lapIndex = sim.lapIndex(:).';

        if numel(lapIndex) < N
            if isempty(lapIndex)
                lapIndex = ones(1, N);
            else
                lapIndex = [lapIndex, lapIndex(end)*ones(1, N - numel(lapIndex))];
            end
        end

        lapIndex = lapIndex(1:N);

        invalid = ~isfinite(lapIndex) | lapIndex <= 0;

        if all(invalid)
            lapIndex = ones(1, N);
        else
            firstValid = find(~invalid, 1, 'first');

            if firstValid > 1
                lapIndex(1:firstValid-1) = lapIndex(firstValid);
            end

            for k = firstValid+1:N
                if invalid(k)
                    lapIndex(k) = lapIndex(k-1);
                end
            end
        end

        lapIndex = round(lapIndex);

    else

        lapIndex = LocalDetectLaps(pos);
    end

    validLap = isfinite(lapIndex) & lapIndex > 0;

    if any(validLap)
        uniqueLaps = unique(lapIndex(validLap));
    else
        uniqueLaps = 1;
        lapIndex = ones(1, N);
    end

    nLaps = numel(uniqueLaps);

    if nLaps < 1
        uniqueLaps = 1;
        nLaps = 1;
        lapIndex = ones(1, N);
    end
end

%% =======================================================================
function lapIndex = LocalDetectLaps(pos)

    N = size(pos, 2);

    if N < 2
        lapIndex = ones(1, N);
        return;
    end

    x = pos(1,:);
    y = pos(2,:);

    xRange = max(x) - min(x);
    yRange = max(y) - min(y);

    if xRange < 1e-9 || yRange < 1e-9
        lapIndex = ones(1, N);
        return;
    end

    valid = isfinite(x) & isfinite(y);

    if nnz(valid) < 2
        lapIndex = ones(1, N);
        return;
    end

    centerX = mean(x(valid));
    centerY = mean(y(valid));

    angle = unwrap(atan2(y - centerY, x - centerX));

    dAngle = diff(angle);
    dAngle = dAngle(isfinite(dAngle) & abs(dAngle) > 1e-9);

    if isempty(dAngle)
        lapIndex = ones(1, N);
        return;
    end

    direction = sign(median(dAngle));

    if direction == 0
        direction = 1;
    end

    progressAngle = direction*(angle - angle(1));
    progressAngle = max(progressAngle, 0);

    lapIndex = floor(max(progressAngle - 1e-9, 0)/(2*pi)) + 1;

    if max(progressAngle) < 2*pi
        lapIndex = ones(1, N);
    end
end

%% =======================================================================
function motorOmega = LocalGetMotorOmega(sim, N)

    motorOmega = [];

    if isfield(sim, 'motorOmega') && ~isempty(sim.motorOmega)

        motorOmega = sim.motorOmega;

    elseif isfield(sim, 'omegaSquared') && ~isempty(sim.omegaSquared)

        motorOmega = sqrt(max(sim.omegaSquared, 0));

    else

        return;
    end

    if size(motorOmega, 1) == 4
        % Formato correto: 4xN.
    elseif size(motorOmega, 2) == 4
        motorOmega = motorOmega.';
    else
        warning('sim.motorOmega deve ter dimensao 4xN ou Nx4.');
        motorOmega = [];
        return;
    end

    if size(motorOmega, 2) < N
        motorOmega = [motorOmega, repmat(motorOmega(:,end), 1, N - size(motorOmega,2))];
    end

    motorOmega = motorOmega(:, 1:N);
end