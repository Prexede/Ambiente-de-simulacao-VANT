function anim = AnimateQuadrotorSimulation(sim, varargin)
% AnimateQuadrotorSimulation
% -------------------------------------------------------------------------
% Anima o movimento 3D do quadrirotor a partir da struct de simulacao.
%
% USO BASICO:
%   AnimateQuadrotorSimulation(sim);
%
% USO RECOMENDADO:
%   AnimateQuadrotorSimulation(sim, ...
%       "Step", 10, ...
%       "DelayMode", "fixed", ...
%       "FixedDelay", 0.03, ...
%       "ColorByLap", true);
%
% SALVAR COMO GIF:
%   AnimateQuadrotorSimulation(sim, ...
%       "SaveGif", true, ...
%       "GifFile", "animacao_quadrirotor.gif", ...
%       "Step", 10, ...
%       "DelayMode", "fixed", ...
%       "FixedDelay", 0.03, ...
%       "ColorByLap", true);
%
% Campos esperados em sim:
%   sim.t              -> vetor 1xN ou Nx1 de tempo [s]
%   sim.state          -> matriz 12xN com estados:
%                         [x y z x_dot y_dot z_dot phi theta psi p q r]'
%   sim.r_des          -> referencia de posicao 3xN ou Nx3, opcional
%   sim.quad.armLength -> comprimento fisico do braco [m], opcional
%
% Opcoes:
%   "SaveGif"          true/false
%   "GifFile"          nome do arquivo .gif
%   "FrameRate"        taxa aproximada de quadros [fps]
%   "PlaybackSpeed"    velocidade da animacao, se DelayMode="simulation"
%   "DelayMode"        "simulation" ou "fixed"
%   "FixedDelay"       atraso fixo entre frames [s]
%   "MinDelay"         atraso minimo entre frames [s]
%   "Step"             passo entre amostras. Ex.: 10 plota 1 a cada 10 pontos
%   "ShowReference"    plota a trajetoria desejada, se existir sim.r_des
%   "ShowTrail"        mostra o rastro percorrido
%   "ColorByLap"       muda a cor do rastro a cada volta
%   "UsePhysicalSize"  true usa tamanho fisico; false aumenta visualmente o drone
%   "ArmLength"        comprimento do braco usado no desenho [m]
%   "View"             vetor [az el] da visualizacao 3D
%   "ExternalFigure"   true abre janela externa

    %% ------------------------- Entradas ---------------------------------
    if nargin < 1 || isempty(sim)
        error('Informe a struct sim gerada pela simulacao.');
    end

    p = inputParser;
    addParameter(p, 'SaveGif', false, @(x) islogical(x) && isscalar(x));
    addParameter(p, 'GifFile', 'animacao_quadrirotor.gif', @(x) ischar(x) || isstring(x));
    addParameter(p, 'FrameRate', 30, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'PlaybackSpeed', 1, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'DelayMode', 'simulation', @(x) any(strcmpi(string(x), ["simulation", "fixed"])));
    addParameter(p, 'FixedDelay', 0.03, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, 'MinDelay', 0.01, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, 'Step', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x >= 1));
    addParameter(p, 'ShowReference', true, @(x) islogical(x) && isscalar(x));
    addParameter(p, 'ShowTrail', true, @(x) islogical(x) && isscalar(x));
    addParameter(p, 'ColorByLap', true, @(x) islogical(x) && isscalar(x));
    addParameter(p, 'UsePhysicalSize', false, @(x) islogical(x) && isscalar(x));
    addParameter(p, 'ArmLength', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
    addParameter(p, 'View', [35 25], @(x) isnumeric(x) && numel(x) == 2);
    addParameter(p, 'ExternalFigure', true, @(x) islogical(x) && isscalar(x));
    parse(p, varargin{:});

    opt = p.Results;
    opt.GifFile = char(opt.GifFile);
    opt.DelayMode = lower(string(opt.DelayMode));

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

    % Aceita tanto 12xN quanto Nx12.
    if size(state, 1) == 12
        % formato esperado
    elseif size(state, 2) == 12
        state = state.';
    else
        error('sim.state deve ter dimensao 12xN ou Nx12.');
    end

    N = min(numel(t), size(state, 2));
    t = t(1:N);
    state = state(:, 1:N);

    pos = state(1:3, :);
    att = state(7:9, :); % [phi; theta; psi]

    hasReference = isfield(sim, 'r_des') && ~isempty(sim.r_des);

    if hasReference
        rDes = LocalEnsure3xN(sim.r_des, 'sim.r_des');
        rDes = rDes(:, 1:min(size(rDes, 2), N));
    else
        rDes = [];
    end

    showReference = opt.ShowReference && hasReference;

    %% ---------------------- Deteccao das voltas -------------------------
    if opt.ColorByLap
        lapIndex = LocalDetectLaps(pos);
        nLaps = max(lapIndex);

        if nLaps < 1 || ~isfinite(nLaps)
            nLaps = 1;
            lapIndex = ones(1, N);
        end

        lapColors = lines(nLaps);
    else
        lapIndex = ones(1, N);
        nLaps = 1;
        lapColors = [0 0.4470 0.7410];
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
        % Escolhe um passo coerente com FrameRate e PlaybackSpeed.
        % Exemplo: Ts=0.01, FrameRate=30, PlaybackSpeed=1 -> step aprox. 3.
        step = max(1, round(opt.PlaybackSpeed/(opt.FrameRate*TsMedio)));
    else
        step = manualStep;
    end

    frameIndex = 1:step:N;

    if frameIndex(end) ~= N
        frameIndex = [frameIndex, N];
    end

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

    xyzRange = max(allXYZ, [], 2) - min(allXYZ, [], 2);
    sceneSpan = max(xyzRange);

    if sceneSpan <= 0 || ~isfinite(sceneSpan)
        sceneSpan = 1;
    end

    if opt.UsePhysicalSize
        L = armPhysical;
    else
        % Aumenta somente a representacao visual para o quadrirotor aparecer
        % bem em trajetorias grandes, como z = 10 m.
        L = max(armPhysical, 0.04*sceneSpan);
    end

    propRadius = 0.28*L;
    zBodyAxisLength = 0.55*L;

    %% ---------------------- Figura e eixos ------------------------------
    if opt.ExternalFigure
        fig = figure('Name', 'Animacao do Quadrirotor', ...
                     'Color', 'w', ...
                     'WindowStyle', 'normal', ...
                     'Visible', 'on', ...
                     'Position', [100 100 1100 750]);

        try
            movegui(fig, 'center');
        catch
            % Algumas versoes/ambientes do MATLAB podem nao suportar movegui.
        end
    else
        fig = figure('Name', 'Animacao do Quadrirotor', ...
                     'Color', 'w', ...
                     'Visible', 'on', ...
                     'Position', [100 100 1100 750]);
    end

    ax = axes(fig);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    axis(ax, 'vis3d');
    view(ax, opt.View(1), opt.View(2));

    xlabel(ax, 'x [m]');
    ylabel(ax, 'y [m]');
    zlabel(ax, 'z [m]');
    title(ax, 'Movimento 3D do quadrirotor');

    % Trajetoria simulada completa.
    if opt.ColorByLap
        for lap = 1:nLaps
            idxLap = find(lapIndex == lap);

            plot3(ax, pos(1,idxLap), pos(2,idxLap), pos(3,idxLap), ':', ...
                  'LineWidth', 1.2, ...
                  'Color', lapColors(lap,:), ...
                  'DisplayName', sprintf('trajetoria volta %d', lap));
        end
    else
        plot3(ax, pos(1,:), pos(2,:), pos(3,:), ':', ...
              'LineWidth', 1.0, ...
              'DisplayName', 'trajetoria simulada');
    end

    % Referencia, caso exista.
    if showReference
        plot3(ax, rDes(1,:), rDes(2,:), rDes(3,:), '--', ...
              'LineWidth', 1.2, ...
              'Color', [0.8500 0.3250 0.0980], ...
              'DisplayName', 'referencia');
    end

    % Limites da cena.
    xyzMin = min(allXYZ, [], 2);
    xyzMax = max(allXYZ, [], 2);
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

    % Rastro animado.
    if opt.ColorByLap
        hTrail = gobjects(nLaps, 1);

        for lap = 1:nLaps
            hTrail(lap) = plot3(ax, nan, nan, nan, ...
                                'LineWidth', 2.0, ...
                                'Color', lapColors(lap,:), ...
                                'DisplayName', sprintf('rastro volta %d', lap));
        end
    else
        hTrail = plot3(ax, nan, nan, nan, ...
                       'LineWidth', 1.5, ...
                       'DisplayName', 'rastro');
    end

    hText = text(ax, xyzMin(1), xyzMin(2), xyzMax(3) + margin*0.5, '', ...
                 'FontWeight', 'bold');

    legend(ax, 'Location', 'bestoutside');

    %% ---------------------- Geometria no sistema do corpo ---------------
    % Convencao visual:
    %
    %   motor 1: -x_b
    %   motor 2: -y_b
    %   motor 3: +x_b
    %   motor 4: +y_b
    %
    % Isso forma a configuracao em cruz do quadrirotor.

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
    pause(0.2); % tempo para a janela externa aparecer antes do loop

    for ii = 1:numel(frameIndex)
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
                for lap = 1:nLaps
                    idxTrail = find(lapIndex(1:k) == lap);

                    set(hTrail(lap), ...
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

        set(hText, 'String', sprintf(['t = %.2f s\n' ...
                                      'phi = %.2f deg,  theta = %.2f deg,  psi = %.2f deg'], ...
                                      t(k), rad2deg(phi), rad2deg(theta), rad2deg(psi)));

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
    anim.numberOfLaps = nLaps;

    if opt.SaveGif
        anim.gifFile = opt.GifFile;
        fprintf('GIF salvo em: %s\n', opt.GifFile);
    end
end

%% =======================================================================
function delay = LocalDelay(ii, frameIndex, t, TsMedio, step, opt)
% Calcula o intervalo de pausa entre frames.

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

    % O formato GIF costuma ignorar atrasos muito pequenos em alguns leitores.
    if opt.SaveGif
        delay = max(delay, 0.02);
    end
end

%% =======================================================================
function R = LocalRotationMatrixZYX(phi, theta, psi)
% Matriz de rotacao corpo -> inercial usando sequencia ZYX:
%
%   R = Rz(psi)*Ry(theta)*Rx(phi)
%
% Estados:
%   phi   = roll
%   theta = pitch
%   psi   = yaw

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
% Aceita dados 3xN ou Nx3 e retorna 3xN.

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
% Soma o vetor coluna b a todas as colunas de A sem depender de expansao
% implicita, mantendo compatibilidade com versoes mais antigas do MATLAB.

    out = bsxfun(@plus, A, b);
end

%% =======================================================================
function lapIndex = LocalDetectLaps(pos)
% Detecta voltas no plano x-y usando o angulo acumulado ao redor do centro
% medio da trajetoria.
%
% Entrada:
%   pos -> matriz 3xN com [x; y; z]
%
% Saida:
%   lapIndex -> vetor 1xN indicando a qual volta cada amostra pertence

    N = size(pos, 2);

    if N < 2
        lapIndex = ones(1, N);
        return;
    end

    x = pos(1,:);
    y = pos(2,:);

    xRange = max(x) - min(x);
    yRange = max(y) - min(y);

    % Se praticamente nao houve movimento no plano x-y, considera uma volta.
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

    % Subtrai uma tolerancia pequena para evitar que o ponto exatamente em
    % 2*pi ja seja classificado como a proxima volta.
    lapIndex = floor(max(progressAngle - 1e-9, 0)/(2*pi)) + 1;

    % Se a trajetoria nao completou uma volta, fica tudo como volta 1.
    if max(progressAngle) < 2*pi
        lapIndex = ones(1, N);
    end
end