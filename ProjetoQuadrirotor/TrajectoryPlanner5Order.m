function traj = TrajectoryPlanner5Order(tipoTrajetoria, plotar, numRepeticoes, varargin)
% TrajectoryPlanner5Order
% -------------------------------------------------------------------------
% Gera uma trajetoria desejada para o quadrirotor a partir de waypoints,
% utilizando interpolacao polinomial de 5a ordem em cada segmento.
%
% Esta versao retorna:
%   traj.t       -> tempo [N x 1]
%   traj.r_des   -> posicao desejada [N x 3]
%   traj.v_des   -> velocidade desejada [N x 3]
%   traj.a_des   -> aceleracao desejada [N x 3]
%   traj.psi_des -> yaw desejado [N x 1]
%
% Observacao:
%   O primeiro ponto dos segmentos seguintes e removido para evitar tempos
%   repetidos em traj.t. Isso evita o erro do interp1:
%       Sample points must be unique.
% -------------------------------------------------------------------------

    %% ---------------------- Tratamento de entradas ----------------------
    if nargin < 1 || isempty(tipoTrajetoria)
        tipoTrajetoria = "quad";
    end

    if nargin < 2 || isempty(plotar)
        plotar = true;
    end

    if nargin < 3 || isempty(numRepeticoes)
        numRepeticoes = 1;
    end

    if numRepeticoes < 1
        error('numRepeticoes deve ser maior ou igual a 1.');
    end

    numRepeticoes = round(numRepeticoes);

    p = inputParser;
    addParameter(p, 'Ts', 0.01, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'TempoSegmento', 5, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'YawDesejado', 0, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'PlotarYaw', false, @(x) islogical(x) && isscalar(x));
    addParameter(p, 'PlotarDerivadas', false, @(x) islogical(x) && isscalar(x));
    parse(p, varargin{:});

    Ts = p.Results.Ts;
    tempoSegmento = p.Results.TempoSegmento;
    yawDesejado = p.Results.YawDesejado;
    plotarYaw = p.Results.PlotarYaw;
    plotarDerivadas = p.Results.PlotarDerivadas;

    %% ---------------------- Definicao dos waypoints ---------------------
    switch lower(string(tipoTrajetoria))

        case "quad"
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0;
               -5.0    0.0   10.0;
               -5.0   -5.0   10.0;
                5.0   -5.0   10.0;
                5.0    5.0   10.0;
               -5.0    5.0   10.0;
               -5.0    0.0   10.0;
                0.0    0.0    0.0
            ];

        case "testz"
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0
            ];

        case "testx"
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0;
                5.0    0.0   10.0
            ];

        case "testy"
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0;
                0.0    5.0   10.0
            ];

        case "testxy"
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0;
                5.0    0.0   10.0;
                5.0    5.0   10.0
            ];

        case "testxyz"
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0;
                5.0    0.0   10.0;
                5.0    5.0   10.0;
                0.0    0.0    0.0
            ];

        case {"hex", "hexagono"}
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0;
                5.0    0.0   10.0;
                2.5    4.33  10.0;
               -2.5    4.33  10.0;
               -5.0    0.0   10.0;
               -2.5   -4.33  10.0;
                2.5   -4.33  10.0;
                5.0    0.0   10.0;
                0.0    0.0    0.0
            ];

        otherwise
            error('Tipo de trajetoria "%s" nao reconhecido.', string(tipoTrajetoria));
    end

    %% ------------------- Repeticao da trajetoria base -------------------
    waypointsRepetidos = construirWaypointsRepetidos(waypoints, numRepeticoes);

    %% ---------------- Gera posicao, velocidade e aceleracao -------------
    nSegmentos = size(waypointsRepetidos, 1) - 1;

    tTotal = [];
    rDesTotal = [];
    vDesTotal = [];
    aDesTotal = [];

    tempoAcumulado = 0;

    for k = 1:nSegmentos

        p0 = waypointsRepetidos(k,   :);
        pf = waypointsRepetidos(k+1, :);

        tLocal = 0:Ts:tempoSegmento;

        if abs(tLocal(end) - tempoSegmento) > 10*eps(max(1, tempoSegmento))
            tLocal = [tLocal, tempoSegmento]; %#ok<AGROW>
        end

        nAmostras = numel(tLocal);
        rSegmento = zeros(nAmostras, 3);
        vSegmento = zeros(nAmostras, 3);
        aSegmento = zeros(nAmostras, 3);

        for eixo = 1:3
            [q, qd, qdd] = quinticaCompleta( ...
                p0(eixo), ...
                pf(eixo), ...
                tempoSegmento, ...
                tLocal ...
            );

            rSegmento(:, eixo) = q(:);
            vSegmento(:, eixo) = qd(:);
            aSegmento(:, eixo) = qdd(:);
        end

        tSegmento = tempoAcumulado + tLocal(:);

        % Remove o primeiro ponto dos segmentos depois do primeiro para nao
        % duplicar o tempo no encontro entre dois segmentos.
        if k > 1
            tSegmento = tSegmento(2:end);
            rSegmento = rSegmento(2:end, :);
            vSegmento = vSegmento(2:end, :);
            aSegmento = aSegmento(2:end, :);
        end

        tTotal = [tTotal; tSegmento]; %#ok<AGROW>
        rDesTotal = [rDesTotal; rSegmento]; %#ok<AGROW>
        vDesTotal = [vDesTotal; vSegmento]; %#ok<AGROW>
        aDesTotal = [aDesTotal; aSegmento]; %#ok<AGROW>

        tempoAcumulado = tempoAcumulado + tempoSegmento;
    end

    %% ---------------------- Verificacao de seguranca --------------------
    if any(diff(tTotal) <= 0)
        error(['TrajectoryPlanner5Order gerou traj.t com tempos repetidos ', ...
               'ou fora de ordem. Verifique Ts, TempoSegmento e numRepeticoes.']);
    end

    psiDes = yawDesejado * ones(size(tTotal));

    %% --------------------------- Saida final ----------------------------
    traj = struct();

    traj.t = tTotal;
    traj.r_des = rDesTotal;
    traj.v_des = vDesTotal;
    traj.a_des = aDesTotal;
    traj.psi_des = psiDes;

    traj.waypoints = waypointsRepetidos;
    traj.Ts = Ts;
    traj.tempoSegmento = tempoSegmento;
    traj.numRepeticoes = numRepeticoes;
    traj.tipoTrajetoria = char(string(tipoTrajetoria));

    %% ------------------------------- Plot -------------------------------
    if plotar
        plotarTrajetoria(traj, plotarYaw, plotarDerivadas);
    end
end

%% ========================================================================
function [q, qd, qdd] = quinticaCompleta(q0, qf, tf, t)
% quinticaCompleta
% -------------------------------------------------------------------------
% Polinomio de 5a ordem com:
%   q(0)    = q0
%   q(tf)   = qf
%   dq(0)   = 0
%   dq(tf)  = 0
%   ddq(0)  = 0
%   ddq(tf) = 0
%
% Retorna posicao, velocidade e aceleracao desejadas.
% -------------------------------------------------------------------------

    a0 = q0;
    a1 = 0;
    a2 = 0;
    a3 = 10 * (qf - q0) / tf^3;
    a4 = -15 * (qf - q0) / tf^4;
    a5 = 6 * (qf - q0) / tf^5;

    q = a0 ...
        + a1*t ...
        + a2*t.^2 ...
        + a3*t.^3 ...
        + a4*t.^4 ...
        + a5*t.^5;

    qd = a1 ...
        + 2*a2*t ...
        + 3*a3*t.^2 ...
        + 4*a4*t.^3 ...
        + 5*a5*t.^4;

    qdd = 2*a2 ...
        + 6*a3*t ...
        + 12*a4*t.^2 ...
        + 20*a5*t.^3;
end

%% ========================================================================
function waypointsOut = construirWaypointsRepetidos(waypointsBase, numRepeticoes)
% construirWaypointsRepetidos
% -------------------------------------------------------------------------
% Repete os waypoints da trajetoria base sem duplicar o primeiro ponto de
% cada repeticao.
% -------------------------------------------------------------------------

    waypointsOut = waypointsBase;

    for i = 2:numRepeticoes
        waypointsOut = [waypointsOut; waypointsBase(2:end, :)]; %#ok<AGROW>
    end
end

%% ========================================================================
function plotarTrajetoria(traj, plotarYaw, plotarDerivadas)
% plotarTrajetoria
% -------------------------------------------------------------------------
% Plota a trajetoria planejada. Opcionalmente plota velocidade e aceleracao
% desejadas.
% -------------------------------------------------------------------------

    figure('Name', 'Trajetoria do Quadrirotor');

    if plotarYaw
        nLinhas = 4;
    else
        nLinhas = 3;
    end

    tiledlayout(nLinhas, 2, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact');

    %% Trajetoria 3D
    nexttile(1, [nLinhas 1]);

    plot3( ...
        traj.r_des(:,1), ...
        traj.r_des(:,2), ...
        traj.r_des(:,3), ...
        'b', ...
        'LineWidth', 1.5 ...
    );

    hold on;

    plot3( ...
        traj.waypoints(:,1), ...
        traj.waypoints(:,2), ...
        traj.waypoints(:,3), ...
        'ro', ...
        'MarkerSize', 6, ...
        'LineWidth', 1.2 ...
    );

    grid on;
    axis equal;

    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');

    title('Trajetoria 3D');
    legend('Trajetoria desejada', 'Waypoints', 'Location', 'best');

    view(3);

    %% Posicao desejada em x
    nexttile(2);
    plot(traj.t, traj.r_des(:,1), 'LineWidth', 1.2);
    grid on;
    ylabel('x_d [m]');
    title('Posicao desejada em x');

    %% Posicao desejada em y
    nexttile(4);
    plot(traj.t, traj.r_des(:,2), 'LineWidth', 1.2);
    grid on;
    ylabel('y_d [m]');
    title('Posicao desejada em y');

    %% Posicao desejada em z
    nexttile(6);
    plot(traj.t, traj.r_des(:,3), 'LineWidth', 1.2);
    grid on;
    ylabel('z_d [m]');
    title('Posicao desejada em z');

    %% Yaw desejado
    if plotarYaw
        nexttile(8);
        plot(traj.t, rad2deg(traj.psi_des), 'LineWidth', 1.2);
        grid on;
        ylabel('\psi_d [graus]');
        xlabel('Tempo [s]');
        title('Yaw desejado');
    else
        xlabel('Tempo [s]');
    end

    %% Plot opcional das derivadas da referencia
    if plotarDerivadas
        figure('Name', 'Derivadas da trajetoria desejada');

        tiledlayout(2, 1, ...
            'TileSpacing', 'compact', ...
            'Padding', 'compact');

        nexttile;
        plot(traj.t, traj.v_des(:,1), 'LineWidth', 1.2);
        hold on;
        plot(traj.t, traj.v_des(:,2), 'LineWidth', 1.2);
        plot(traj.t, traj.v_des(:,3), 'LineWidth', 1.2);
        grid on;
        xlabel('Tempo [s]');
        ylabel('v_d [m/s]');
        title('Velocidade desejada');
        legend('v_x_d', 'v_y_d', 'v_z_d', 'Location', 'best');

        nexttile;
        plot(traj.t, traj.a_des(:,1), 'LineWidth', 1.2);
        hold on;
        plot(traj.t, traj.a_des(:,2), 'LineWidth', 1.2);
        plot(traj.t, traj.a_des(:,3), 'LineWidth', 1.2);
        grid on;
        xlabel('Tempo [s]');
        ylabel('a_d [m/s^2]');
        title('Aceleracao desejada');
        legend('a_x_d', 'a_y_d', 'a_z_d', 'Location', 'best');
    end
end
