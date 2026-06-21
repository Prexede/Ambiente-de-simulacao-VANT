%% main.m
% -------------------------------------------------------------------------
% Simulacao organizada do quadrirotor
%
% Sequencia:
% 1 - Planejador de trajetoria
% 2 - Criacao do modelo do quadrotor
% 3 - Configuracao de disturbios
% 4 - Configuracao da simulacao
% 5 - Configuracao dos controladores
% 6 - Loop principal
% 7 - Plots / animacao
% -------------------------------------------------------------------------

clear;
close all;
clc;

%% ========================================================================
% 1 - Planejador de trajetoria
% ========================================================================
% Aqui ficam todas as informacoes da trajetoria.
% A trajetoria e gerada primeiro porque dela vem o tempo final da simulacao.

trajConfig = struct();

trajConfig.type         = "quad";   % "quad", "testZ", "testX", "testY", "testXY", "testXYZ"
trajConfig.repetitions  = 3;        % numero de voltas/repeticoes
trajConfig.segmentTime  = 10;        % tempo de cada segmento [s]
trajConfig.yawDesired   = 0;        % yaw desejado [rad]
trajConfig.Ts           = 0.01;     % passo de amostragem da trajetoria [s]
trajConfig.plot         = false;     % plota somente a trajetoria planejada

traj = TrajectoryPlanner5Order( ...
    trajConfig.type, ...
    trajConfig.plot, ...
    trajConfig.repetitions, ...
    "Ts", trajConfig.Ts, ...
    "TempoSegmento", trajConfig.segmentTime, ...
    "YawDesejado", trajConfig.yawDesired);

% Guarda a trajetoria dentro da configuracao para o loop principal usar.
trajConfig.traj = traj;

% Tempo total real da trajetoria.
trajConfig.tf = traj.t(end);

% Duracao de cada volta.
trajConfig.lapDuration = traj.t(end) / trajConfig.repetitions;

fprintf('\n--- Trajetoria ---\n');
fprintf('Tipo: %s\n', trajConfig.type);
fprintf('Numero de repeticoes: %d\n', trajConfig.repetitions);
fprintf('Tempo de cada segmento: %.2f s\n', trajConfig.segmentTime);
fprintf('Tempo total da trajetoria: %.2f s\n', trajConfig.tf);

%% ========================================================================
% 2 - Criacao do modelo do quadrotor
% ========================================================================
% Aqui entram as informacoes fisicas usadas para estimar massa, inercia,
% comprimento de braco e coeficientes aerodinamicos.

quadConfig = struct();

quadConfig.material  = "CarbonFiber";
quadConfig.droneGeom = [20 5 1];    % [L W H] em centimetros
quadConfig.w_hover   = 1000;        % velocidade angular de hover [rad/s]

[I_til, quadNominal] = EstimatedQuadParameters( ...
    quadConfig.material, ...
    quadConfig.droneGeom, ...
    quadConfig.w_hover);

% Guarda o modelo ja estimado para evitar recalcular dentro do loop.
quadConfig.I_til = I_til;
quadConfig.quadNominal = quadNominal;

fprintf('\n--- Modelo do quadrotor ---\n');
fprintf('Material: %s\n', quadConfig.material);
fprintf('Geometria [L W H]: [%.2f %.2f %.2f] cm\n', quadConfig.droneGeom);
fprintf('Massa nominal: %.6f kg\n', quadNominal.mass);
fprintf('Comprimento efetivo do braco: %.6f m\n', quadNominal.armLength);
fprintf('Tensor de inercia [kg.m^2]:\n');
disp(I_til);

%% ========================================================================
% 3 - Configuracao de disturbios
% ========================================================================
% Modos disponiveis:
%   "Nominal"      -> sem disturbio
%   "MassChange"  -> muda massa por volta
%   "Wind"        -> aplica vento por volta
%   "MassWind"    -> muda massa e aplica vento
%   "TrajectoryOnly" -> apenas plota/gera a trajetoria

flightConfig = struct();

flightConfig.mode = "Wind";

% Numero de voltas usado pelo gerenciador de condicoes de voo.
flightConfig.numLaps = trajConfig.repetitions;
flightConfig.lapDuration = trajConfig.lapDuration;

% -------------------------------------------------------------------------
% Mudanca de massa
% -------------------------------------------------------------------------
% Formato:
%   [volta, delta_massa_kg]

flightConfig.mass.enabled = true;

flightConfig.mass.byLap = [
    1,  0.00;
    2, -0.25;
    3, -0.5
];

% -------------------------------------------------------------------------
% Vento
% -------------------------------------------------------------------------
% Formato:
%   [volta, Fx_N, Fy_N, Fz_N]

flightConfig.wind.enabled = true;
flightConfig.wind.referenceFrame = "inertial";

flightConfig.wind.byLap = [
    1, 0.0, 0.0, 0.0;
    2, 1.2, 0.0, 0.0;
    3, 0.5, 0.5, 0.0
];

fprintf('\n--- Condicoes de voo ---\n');
fprintf('Modo: %s\n', flightConfig.mode);
fprintf('Duracao de cada volta: %.2f s\n', flightConfig.lapDuration);

%% ========================================================================
% 4 - Configuracao da simulacao
% ========================================================================
% O tempo final da simulacao deve vir da trajetoria.
% Assim, se voce mudar numero de repeticoes ou tempo de segmento,
% a simulacao acompanha automaticamente.

simConfig = struct();

simConfig.method = "RK4";           % "Euler" ou "RK4"
simConfig.Ts     = trajConfig.Ts;   % passo da simulacao [s]
simConfig.tf     = traj.t(end);     % tempo final automatico [s]

% Estado inicial:
% state = [x y z x_dot y_dot z_dot phi theta psi p q r]'
simConfig.initialState = zeros(12,1);

fprintf('\n--- Simulacao ---\n');
fprintf('Metodo de integracao: %s\n', simConfig.method);
fprintf('Ts: %.4f s\n', simConfig.Ts);
fprintf('tf: %.2f s\n', simConfig.tf);

%% ========================================================================
% 5 - Configuracao dos controladores
% ========================================================================
% Fluxo simples dos ganhos:
%
%   controlConfig.gainSource = "arquivo"
%       -> usa PositionPIDGains.m e AttitudePIDGains.m
%
%   controlConfig.gainSource = "estimador"
%       -> calcula ganhos PD/PID usando tempo de pico e estabilizacao
%
% O vetor gainSpec tem o formato:
%   [tp_pos, ts_pos, tp_att, ts_att]
%
% onde:
%   tp_pos, ts_pos -> usados igualmente para x, y, z
%   tp_att, ts_att -> usados igualmente para phi, theta, psi

controlConfig = struct();

% -------------------------------------------------------------------------
% Escolha dos controladores
% -------------------------------------------------------------------------
controlConfig.position.type = "PID";       % "P", "PD" ou "PID"
controlConfig.attitude.type = "PD";       % "P", "PD" ou "PID"

% -------------------------------------------------------------------------
% Frequencia dos controladores
% -------------------------------------------------------------------------
controlConfig.position.updateFreq = 10;   % [Hz]
controlConfig.attitude.updateFreq = 100;  % [Hz]

% -------------------------------------------------------------------------
% Origem dos ganhos
% -------------------------------------------------------------------------
% "arquivo"   -> usa os arquivos de ganhos ja existentes
% "estimador" -> calcula ganhos por tempo de pico e estabilizacao
% -------------------------------------------------------------------------
controlConfig.gainSource = "arquivo";

% -------------------------------------------------------------------------
% Especificacao usada apenas quando gainSource = "estimador"
% -------------------------------------------------------------------------
controlConfig.gainSpec = [
    1.00, 2.00, ...
    0.12, 0.25
];

% Parametros auxiliares do estimador
controlConfig.settlingFactor = 4.0;  % 3.0 ~= 5%, 4.0 ~= 2%, 4.6 ~= 1%
controlConfig.pidGamma = 5;          % usado apenas no PID

fprintf('\n--- 5) Controladores ---\n');
fprintf('Posicao: %s @ %.3f Hz\n', ...
    controlConfig.position.type, controlConfig.position.updateFreq);
fprintf('Atitude: %s @ %.3f Hz\n', ...
    controlConfig.attitude.type, controlConfig.attitude.updateFreq);
fprintf('Origem dos ganhos: %s\n', controlConfig.gainSource);

%% ========================================================================
% 6 - Loop principal
% ========================================================================

plotSimulation = true;

sim = ControlledMainLoop( ...
    quadConfig, ...
    trajConfig, ...
    simConfig, ...
    controlConfig, ...
    flightConfig, ...
    plotSimulation);

%% ========================================================================
% 7 - Plots / Animacao
% ========================================================================

runAnimation = true;

if runAnimation
    AnimateQuadrotorSimulation(sim, ...
        "Step", 10, ...
        "DelayMode", "fixed", ...
        "FixedDelay", 0.03, ...
        "ColorByLap", true, ...
        "Theme", "dark");
end

fprintf('\nSimulacao finalizada.\n');
