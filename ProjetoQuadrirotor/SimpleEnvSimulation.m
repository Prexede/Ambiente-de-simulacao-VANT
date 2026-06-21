function sim = SimpleEnvSimulation(method, simConfig, quad, plotData)
% SimpleEnvSimulation
% -------------------------------------------------------------------------
% Simulacao simples do ambiente dinamico do quadrirotor, sem controladores.
%
% Esta funcao utiliza:
%   - QuadrotorDynamics.m para calcular a derivada dos estados;
%   - metodo de Euler ou Runge-Kutta de 4a ordem para integrar os estados.
%
% ENTRADAS:
%   method    : metodo de integracao
%               "Euler" ou "RK4"
%
%   simConfig : struct com as configuracoes da simulacao
%               simConfig.Ts         -> passo de simulacao [s]
%               simConfig.tf         -> tempo final [s]
%               simConfig.motorOmega -> vetor 4x1 com velocidades dos motores [rad/s]
%
%   quad      : struct gerada por EstimatedQuadParameters
%
%   plotData  : true/false para plotar os dados ao final da simulacao
%
% SAIDA:
%   sim : struct com os dados da simulacao
%         sim.t          -> vetor de tempo
%         sim.state      -> matriz 12xN com os estados simulados
%         sim.stateDot   -> matriz 12xN com as derivadas dos estados
%         sim.method     -> metodo utilizado
%         sim.motorOmega -> entrada dos motores utilizada
%         sim.Ts         -> passo de simulacao
%         sim.tf         -> tempo final
%
% EXEMPLO:
%   material = "CarbonFiber";
%   droneGeom = [20 5 1];
%   w_hover = 1000;
%
%   [I_til, quad] = EstimatedQuadParameters(material, droneGeom, w_hover);
%
%   simConfig.Ts = 0.01;
%   simConfig.tf = 5;
%   simConfig.motorOmega = quad.hoverOmega*ones(4,1);
%
%   sim = SimpleEnvSimulation("Euler", simConfig, quad, true);
%
% -------------------------------------------------------------------------

    %% ---------------------- Tratamento minimo de entrada ----------------
    if nargin < 1 || isempty(method)
        error('Informe o metodo de integracao: "Euler" ou "RK4".');
    end

    if nargin < 2 || isempty(simConfig)
        error('Informe simConfig com Ts, tf e motorOmega.');
    end

    if nargin < 3 || isempty(quad)
        error('Informe a struct quad gerada por EstimatedQuadParameters.');
    end

    if nargin < 4 || isempty(plotData)
        error('Informe plotData como true ou false.');
    end

    %% ---------------------- Configuracoes da simulacao ------------------
    Ts = simConfig.Ts;
    tf = simConfig.tf;
    motorOmega = simConfig.motorOmega;

    t = 0:Ts:tf;
    N = length(t);

    %% ---------------------- Estado inicial ------------------------------
    state = zeros(12, N);
    stateDot = zeros(12, N);

    %% ---------------------- Loop de simulacao ---------------------------
    for k = 1:N-1

        currentState = state(:,k);

        switch method
            case "Euler"
                currentStateDot = QuadrotorDynamics(currentState, motorOmega, quad);
                nextState = currentState + Ts*currentStateDot;

            case "RK4"
                currentStateDot = QuadrotorDynamics(currentState, motorOmega, quad);
                nextState = RungeKutta4(currentState, motorOmega, quad, Ts);

            otherwise
                error('Metodo nao implementado. Use "Euler" ou "RK4".');
        end

        stateDot(:,k) = currentStateDot;
        state(:,k+1) = nextState;
    end

    stateDot(:,N) = QuadrotorDynamics(state(:,N), motorOmega, quad);

    %% ---------------------- Struct de saida -----------------------------
    sim = struct();

    sim.t = t;
    sim.state = state;
    sim.stateDot = stateDot;
    sim.method = method;
    sim.motorOmega = motorOmega;
    sim.Ts = Ts;
    sim.tf = tf;

    %% ---------------------- Plot dos dados ------------------------------
    if plotData
        PlotSimpleSimulation(sim);
    end
end

%% =======================================================================
function stateNext = RungeKutta4(state, motorOmega, quad, Ts)
% RungeKutta4
% -------------------------------------------------------------------------
% Integra um passo da dinamica usando Runge-Kutta de 4a ordem.
%
% Durante um passo Ts, considera-se motorOmega constante.

    k1 = QuadrotorDynamics(state, motorOmega, quad);
    k2 = QuadrotorDynamics(state + 0.5*Ts*k1, motorOmega, quad);
    k3 = QuadrotorDynamics(state + 0.5*Ts*k2, motorOmega, quad);
    k4 = QuadrotorDynamics(state + Ts*k3, motorOmega, quad);

    stateNext = state + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end

%% =======================================================================
function PlotSimpleSimulation(sim)
% PlotSimpleSimulation
% -------------------------------------------------------------------------
% Plota somente:
%   - trajetoria 3D na esquerda;
%   - posicoes x, y, z na direita superior;
%   - velocidades lineares x_dot, y_dot, z_dot na direita inferior.

    t = sim.t;
    state = sim.state;

    x = state(1,:);
    y = state(2,:);
    z = state(3,:);

    x_dot = state(4,:);
    y_dot = state(5,:);
    z_dot = state(6,:);

    figure('Name', 'Simple Environment Simulation');

    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    %% Trajetoria 3D na esquerda
    nexttile(1, [2 1]);
    plot3(x, y, z, 'LineWidth', 1.5);
    grid on;
    axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Trajetoria 3D');
    view(3);

    %% Posicoes na direita superior
    nexttile(2);
    plot(t, x, 'LineWidth', 1.2);
    hold on;
    plot(t, y, 'LineWidth', 1.2);
    plot(t, z, 'LineWidth', 1.2);
    grid on;
    xlabel('Tempo [s]');
    ylabel('Posicao [m]');
    title('Posicoes');
    legend('x', 'y', 'z', 'Location', 'best');

    %% Velocidades lineares na direita inferior
    nexttile(4);
    plot(t, x_dot, 'LineWidth', 1.2);
    hold on;
    plot(t, y_dot, 'LineWidth', 1.2);
    plot(t, z_dot, 'LineWidth', 1.2);
    grid on;
    xlabel('Tempo [s]');
    ylabel('Velocidade [m/s]');
    title('Velocidades lineares');
    legend('x dot', 'y dot', 'z dot', 'Location', 'best');
end