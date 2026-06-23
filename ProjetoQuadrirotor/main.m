%% main.m
% -------------------------------------------------------------------------
% Ambiente refatorado para simulacao de voo de um VANT quadrotor.
%
% Fluxo principal:
%   1) Modelo nominal do quadrotor
%   2) Configuracao da simulacao
%   3) Plano de voo: trajetoria + disturbios por volta
%   4) Configuracao dos controladores
%   5) Loop unico de simulacao
%   6) Plots selecionados
% -------------------------------------------------------------------------

clear;
close all;
clc;

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot));

%% ========================================================================
% OPCOES GERAIS DISPONIVEIS
% ========================================================================
% Modelo do quadrotor:
%   Material:
%       "CarbonFiber"
%
% Simulacao / trajetoria:
%   TrajectoryType:
%       "quad"       -> trajetoria quadrada/principal do projeto
%       "testz"      -> subida vertical simples
%       "testx"      -> subida + deslocamento em x
%       "testy"      -> subida + deslocamento em y
%       "testxy"     -> subida + deslocamento em x e y
%       "testxyz"    -> teste em x, y e z
%       "hex"        -> trajetoria hexagonal
%       "hexagon"    -> mesmo que "hex"
%
%   IntegrationMethod:
%       "RK4"
%       "Euler"
%
% Disturbios por volta:
%   DisturbanceMode:
%       "nominal"    -> sem disturbios
%       "mass"       -> variacao de massa por volta
%       "wind"       -> forca externa/vento por volta
%       "masswind"   -> variacao de massa + vento por volta
%
% Controladores:
%   PositionType:
%       "P", "PD", "PID"
%   AttitudeType:
%       "P", "PD", "PID"
%
% Plots:
%   Todos os campos aceitam true ou false:
%       Trajectory, States, Errors, Mass, Motors, Animation

%% ========================================================================
% 1 - Modelo nominal do quadrotor
% ========================================================================
% Parametros aceitos por QuadrotorModel:
%
%   "Material"          -> string. Atualmente implementado: "CarbonFiber".
%   "Geometry"          -> vetor [L W H] em cm.
%   "HoverSpeed"        -> velocidade angular de hover [rad/s].
%   "OmegaMin"          -> limite inferior dos motores [rad/s].
%   "OmegaMax"          -> limite superior dos motores [rad/s].
%   "MaxTiltAngle"      -> inclinacao maxima permitida [rad].
%   "MaxThrustFactor"   -> fator para limite superior de empuxo.
%
% Valores como OmegaMax, MaxTiltAngle e MaxThrustFactor ficam no modelo.
% Caso o usuario informe um valor, ele sobrescreve o valor padrao.

quadConfig = QuadrotorModel( ...
    "Material", "CarbonFiber", ...
    "Geometry", [20 5 1], ...          % [L W H] em cm
    "HoverSpeed", 1000, ...            % [rad/s]
    "OmegaMin", 0, ...                 % [rad/s]
    "OmegaMax", 5000, ...              % [rad/s]
    "MaxTiltAngle", deg2rad(20), ...   % [rad]
    "MaxThrustFactor", 2.5);           % Tmax = fator*m*g

fprintf('\n--- Modelo nominal ---\n');
fprintf('Material: %s\n', quadConfig.material);
fprintf('Massa nominal: %.6f kg\n', quadConfig.mass);
fprintf('Braco efetivo: %.6f m\n', quadConfig.armLength);
fprintf('Omega maximo: %.2f rad/s\n', quadConfig.actuator.omegaMax);
fprintf('Angulo maximo: %.2f graus\n', rad2deg(quadConfig.control.maxTiltAngle));
fprintf('Fator de empuxo maximo: %.2f\n', quadConfig.control.maxThrustFactor);

%% ========================================================================
% 2 - Configuracao da simulacao
% ========================================================================
% Parametros aceitos por SimulationPlanner:
%
%   "TrajectoryType"      -> "quad", "testz", "testx", "testy", "testxy",
%                            "testxyz", "hex" ou "hexagon".
%   "Ts"                  -> passo de simulacao [s].
%   "SegmentTime"         -> tempo para ir de um waypoint ao proximo [s].
%   "Repetitions"         -> numero de voltas completas.
%   "YawDesired"          -> yaw desejado da trajetoria [rad].
%   "IntegrationMethod"   -> "RK4" ou "Euler".
%   "InitialState"        -> vetor de estado inicial 12x1.
%
% Conceitos:
%   waypoint       -> ponto da trajetoria.
%   segmento       -> movimento do waypoint i para o waypoint i+1.
%   volta          -> execucao completa de todos os segmentos da trajetoria.
%   segmentTime    -> tempo de um segmento.
%   lapTime        -> tempo de uma volta completa.

simConfig = SimulationPlanner( ...
    "TrajectoryType", "quad", ...
    "Ts", 0.01, ...
    "SegmentTime", 20, ...
    "Repetitions", 3, ...
    "YawDesired", 0, ...
    "IntegrationMethod", "euler", ...
    "InitialState", zeros(12,1), ...
    "ResetStateEachLap", true);

fprintf('\n--- Simulacao ---\n');
fprintf('Trajetoria: %s\n', simConfig.trajectory.type);
fprintf('Waypoints por volta: %d\n', simConfig.trajectory.numWaypointsPerLap);
fprintf('Segmentos por volta: %d\n', simConfig.trajectory.numSegmentsPerLap);
fprintf('Tempo por segmento: %.2f s\n', simConfig.time.segmentTime);
fprintf('Tempo por volta: %.2f s\n', simConfig.time.lapTime);
fprintf('Numero de voltas: %d\n', simConfig.time.repetitions);
fprintf('Tempo final: %.2f s\n', simConfig.time.tf);
fprintf('Metodo: %s\n', simConfig.integration.method);

%% ========================================================================
% 3 - Plano de voo: trajetoria + disturbios por volta
% ========================================================================
% Parametros aceitos por FlightPlanBuilder:
%
%   "DisturbanceMode" -> define quais disturbios serao usados.
%       "nominal"    -> sem disturbios
%       "mass"       -> variacao de massa
%       "wind"       -> vento/forca externa inercial
%       "masswind"   -> variacao de massa + vento
%
%   "MassByLap" usa o formato:
%       [volta, delta_massa_kg]
%
%   "WindByLap" usa o formato:
%       [volta, Fx_N, Fy_N, Fz_N]
%
% Observacao:
%   Os disturbios sao definidos por volta, nao por segmento.
%   A mudanca de massa sempre e interpretada como delta em relacao a massa
%   nominal do quadrotor.
%   O vento sempre e interpretado como forca externa em Newton no
%   referencial inercial.

flightPlan = FlightPlanBuilder( ...
    simConfig, ...
    quadConfig, ...
    "DisturbanceMode", "nominal", ...
    "MassByLap", [ ...
        1,  0.00; ...
        2, -0.50; ...
        3, -0.50; ...
        4, -0.50; ...
        5, -0.50], ...
    "WindByLap", [ ...
        1, 0.0, 0.0, 0.0; ...
        2, 0.5, 0.0, 0.0; ...
        3, 0.5, 0.5, 0.0]);

fprintf('\n--- Plano de voo ---\n');
fprintf('Modo de disturbio: %s\n', flightPlan.disturbance.mode);
fprintf('Amostras: %d\n', numel(flightPlan.t));

%% ========================================================================
% 3.1 - Disturbio de medicao
% ========================================================================
% Este disturbio nao altera a dinamica real do drone.
% Ele altera apenas os estados medidos usados pelos controladores.
%
% Estados disponiveis:
%   r     -> [x; y; z]
%   v     -> [vx; vy; vz]
%   eta   -> [phi; theta; psi]
%   omega -> [p; q; r]
%
% Tipo disponivel nesta versao:
%   "whiteNoise"

noiseParams = NoiseParams();

noiseParams.enable = true;
noiseParams.disturbanceType = "whiteNoise";

noiseParams.r.enable     = true;    % x, y, z
noiseParams.v.enable     = true;   % vx, vy, vz
noiseParams.eta.enable   = true;    % phi, theta, psi
noiseParams.omega.enable = true;   % p, q, r

fprintf('\n--- Disturbio de medicao ---\n');
fprintf('Ativo: %d\n', noiseParams.enable);
fprintf('Tipo: %s\n', noiseParams.disturbanceType);
fprintf('r [x y z]: %d\n', noiseParams.r.enable);
fprintf('v [vx vy vz]: %d\n', noiseParams.v.enable);
fprintf('eta [phi theta psi]: %d\n', noiseParams.eta.enable);
fprintf('omega [p q r]: %d\n', noiseParams.omega.enable);

%% ========================================================================
% 4 - Controladores
% ========================================================================
% Parametros aceitos por ControllersConfig:
%
%   "PositionType"         -> "P", "PD" ou "PID".
%   "AttitudeType"         -> "P", "PD" ou "PID".
%   "PositionFrequency"    -> frequencia de atualizacao do controle de posicao [Hz].
%   "AttitudeFrequency"    -> frequencia de atualizacao do controle de atitude [Hz].
%
% Observacao:
%   Os ganhos sempre sao carregados dos arquivos:
%       PositionGains.m
%       AttitudeGains.m

controllersConfig = ControllersConfig( ...
    "PositionType", "PD", ...
    "AttitudeType", "PD", ...
    "PositionFrequency", 10, ...
    "AttitudeFrequency", 100);

fprintf('\n--- Controladores ---\n');
fprintf('Posicao: %s @ %.2f Hz\n', controllersConfig.position.type, controllersConfig.position.updateFrequency);
fprintf('Atitude: %s @ %.2f Hz\n', controllersConfig.attitude.type, controllersConfig.attitude.updateFrequency);

%% ========================================================================
% 5 - Loop unico de simulacao
% ========================================================================

simData = SimulationLoop( ...
    simConfig, ...
    quadConfig, ...
    controllersConfig, ...
    flightPlan, ...
    noiseParams);

%% ========================================================================
% 6 - Plots
% ========================================================================
% Parametros aceitos por PlotConfig:
%
%   "Trajectory"        -> true/false. Plota trajetoria 3D nominal e realizada.
%   "States"            -> true/false. Plota estados no tempo.
%   "Errors"            -> true/false. Plota erros de rastreamento.
%   "Mass"              -> true/false. Plota massa ao longo do tempo.
%   "Motors"            -> true/false. Plota velocidades dos motores.
%   "Animation"         -> true/false. Executa animacao 3D.
%   "AnimationStep"     -> inteiro >= 1. Pula amostras na animacao.
%   "AnimationPeriod"   -> periodo do timer da animacao [s].
%
% Observacao:
%   A animacao roda com timer, entao ela nao prende o MATLAB em um loop
%   continuo. Use os botoes da propria figura para pausar, parar ou reiniciar.

plotConfig = PlotConfig( ...
    "Trajectory", true, ...
    "States", true, ...
    "Errors", true, ...
    "Mass", true, ...
    "Motors", true, ...
    "LapComparison", true, ...
    "Animation", true, ...
    "AnimationStep", 20, ...
    "AnimationPeriod", 0.05);

PlotSimulation(simData, plotConfig);