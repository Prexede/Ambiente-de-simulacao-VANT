function sim = ControlledMainLoop(quadConfig, trajConfig, simConfig, controlConfig, flightConfig, plotData)
% ControlledMainLoop
% -------------------------------------------------------------------------
% Gerencia o loop principal da simulacao controlada do quadrirotor.
%
% Versao modificada para aceitar condicoes de voo por volta:
%   - simulacao nominal;
%   - mudanca de massa por volta;
%   - forca externa de vento por volta;
%   - massa + vento por volta.
%
% Assinatura recomendada:
%   sim = ControlledMainLoop(quadConfig, trajConfig, simConfig, ...
%                            controlConfig, flightConfig, plotData)
%
% Compatibilidade:
%   Tambem aceita a chamada antiga:
%   sim = ControlledMainLoop(quadConfig, trajConfig, simConfig, ...
%                            controlConfig, plotData)
%
%   E tambem aceita, caso voce tenha colocado no main nesta ordem:
%   sim = ControlledMainLoop(quadConfig, trajConfig, simConfig, ...
%                            controlConfig, plotData, flightConfig)
%
% ENTRADAS PRINCIPAIS:
%   quadConfig.material     -> material do frame. Ex.: "CarbonFiber"
%   quadConfig.droneGeom    -> vetor [L W H], em centimetros
%   quadConfig.w_hover      -> velocidade angular nominal de hover [rad/s]
%
%   trajConfig.type         -> tipo de trajetoria. Ex.: "Hex"
%   trajConfig.plot         -> true/false para plotar a trajetoria planejada
%   trajConfig.repetitions  -> numero de repeticoes da trajetoria
%   trajConfig.segmentTime  -> tempo de cada segmento da trajetoria [s]
%   trajConfig.yawDesired   -> yaw desejado constante [rad]
%
%   simConfig.method        -> metodo de integracao: "Euler" ou "RK4"
%   simConfig.Ts            -> passo de simulacao [s]
%   simConfig.tf            -> tempo final de simulacao [s]
%
%   controlConfig.position.type       -> "P", "PD" ou "PID"
%   controlConfig.position.updateFreq -> frequencia do controle de posicao [Hz]
%
%   controlConfig.attitude.type       -> "P", "PD" ou "PID"
%   controlConfig.attitude.updateFreq -> frequencia do controle de atitude [Hz]
%
%   flightConfig.mode       -> "Nominal", "MassChange", "Wind" ou "MassWind"
%   plotData                -> true/false para plotar resultado da simulacao
%
% EXEMPLO DE flightConfig:
%   flightConfig.mode = "MassWind";
%
%   flightConfig.mass.enabled = true;
%   flightConfig.mass.byLap = [
%       1, 1.20;
%       2, 1.10;
%       3, 0.95
%   ];
%
%   flightConfig.wind.enabled = true;
%   flightConfig.wind.byLap = [
%       1, 0.0, 0.0, 0.0;
%       2, 0.5, 0.0, 0.0;
%       3, 0.0, 0.5, 0.0
%   ];
%
% OBSERVACAO IMPORTANTE:
%   Para usar vento, o arquivo QuadrotorDynamics.m deve aceitar a chamada:
%       QuadrotorDynamics(state, motorOmega, quadPlant, disturbance)
%
%   A massa ja funciona mesmo que QuadrotorDynamics.m ainda use apenas:
%       QuadrotorDynamics(state, motorOmega, quadPlant)
% -------------------------------------------------------------------------

    %% ---------------------- Compatibilidade de entrada ------------------
    if nargin < 1 || isempty(quadConfig)
        error('Informe quadConfig.');
    end

    if nargin < 2 || isempty(trajConfig)
        error('Informe trajConfig.');
    end

    if nargin < 3 || isempty(simConfig)
        error('Informe simConfig.');
    end

    if nargin < 4 || isempty(controlConfig)
        error('Informe controlConfig.');
    end

    % Caso antigo:
    % ControlledMainLoop(..., controlConfig, plotData)
    %
    % Nesta situacao, o modo de voo pode vir de simConfig.flightMode.
    if nargin == 5
        plotData = flightConfig;
        flightConfig = DefaultFlightConfig(simConfig);
    end

    if nargin < 6 || isempty(plotData)
        plotData = true;
    end

    % Caso o main tenha ficado na ordem:
    % ControlledMainLoop(..., controlConfig, plotData, flightConfig)
    if IsScalarLogicalLike(flightConfig) && isstruct(plotData)
        tempFlightConfig = plotData;
        plotData = logical(flightConfig);
        flightConfig = tempFlightConfig;
    end

    if isempty(flightConfig)
        flightConfig = DefaultFlightConfig(simConfig);
    end

    if ~isstruct(flightConfig)
        error('flightConfig deve ser uma struct.');
    end

    flightConfig = NormalizeFlightConfig(flightConfig);

    % O modo TrajectoryOnly pode ser tratado diretamente aqui para evitar
    % erro caso o main ainda chame ControlledMainLoop em todos os modos.
    plotData = logical(plotData);

    %% ---------------------- Parametros do quadrirotor -------------------
    % Preferencia da nova arquitetura:
    %   main.m calcula o modelo uma vez e envia quadConfig.quadNominal.
    %
    % Compatibilidade:
    %   se quadConfig.quadNominal nao existir, este loop ainda calcula o
    %   modelo usando EstimatedQuadParameters, como nas versoes antigas.

    if isfield(quadConfig, 'quadNominal') && ~isempty(quadConfig.quadNominal)
        quadNominal = quadConfig.quadNominal;

        if isfield(quadConfig, 'I_til') && ~isempty(quadConfig.I_til)
            I_til = quadConfig.I_til;
        elseif isfield(quadNominal, 'Inertia') && ~isempty(quadNominal.Inertia)
            I_til = quadNominal.Inertia;
        else
            error('quadConfig.quadNominal existe, mas nao foi encontrado I_til ou quadNominal.Inertia.');
        end
    else
        [I_til, quadNominal] = EstimatedQuadParameters( ...
            quadConfig.material, ...
            quadConfig.droneGeom, ...
            quadConfig.w_hover);
    end

    % Modelo usado pelo controlador.
    % Este modelo permanece nominal mesmo quando a planta muda.
    quadCtrl = quadNominal;

    %% ---------------------- Planejador de trajetoria --------------------
    % Preferencia da nova arquitetura:
    %   main.m gera a trajetoria uma vez e envia trajConfig.traj.
    %
    % Compatibilidade:
    %   se trajConfig.traj nao existir, este loop ainda chama o planejador.

    if isfield(trajConfig, 'traj') && ~isempty(trajConfig.traj)
        traj = trajConfig.traj;
    else
        traj = TrajectoryPlanner5Order( ...
            trajConfig.type, ...
            trajConfig.plot, ...
            trajConfig.repetitions, ...
            "Ts", simConfig.Ts, ...
            "TempoSegmento", trajConfig.segmentTime, ...
            "YawDesejado", trajConfig.yawDesired);
    end

    traj = ValidateTrajectoryStruct(traj);

    %% ---------------------- Modo apenas trajetoria ----------------------
    if IsTrajectoryOnlyMode(flightConfig.mode)
        sim = BuildTrajectoryOnlyOutput( ...
            traj, quadNominal, I_til, quadConfig, trajConfig, simConfig, ...
            controlConfig, flightConfig);

        if plotData
            PlotTrajectoryOnlySimulation(sim);
        end

        return;
    end

    %% ---------------------- Ganhos dos controladores --------------------
    [posGains, attGains, gainInfo] = GetControllerGains( ...
        quadCtrl, ...
        controlConfig);

    %% ---------------------- Configuracoes da simulacao ------------------
    Ts = simConfig.Ts;
    tf = simConfig.tf;
    method = string(simConfig.method);

    t = 0:Ts:tf;

    % Evita perder o ultimo instante caso tf nao seja multiplo exato de Ts.
    % Neste caso, o passo de integracao continua sendo Ts; portanto, o ideal
    % e manter simConfig.tf vindo de traj.t(end), como feito no main.m.
    if t(end) < tf
        warning(['simConfig.tf nao e multiplo exato de simConfig.Ts. ', ...
                 'O ultimo instante sera aproximado para manter passo constante.']);
    end

    N = length(t);

    %% ---------------------- Frequencias de controle ---------------------
    positionUpdateFreq = controlConfig.position.updateFreq;
    attitudeUpdateFreq = controlConfig.attitude.updateFreq;

    N_pos = max(1, round(1/(positionUpdateFreq*Ts)));
    N_att = max(1, round(1/(attitudeUpdateFreq*Ts)));

    %% ---------------------- Vetores de simulacao ------------------------
    state = zeros(12, N);
    stateDot = zeros(12, N);

    % Estado inicial opcional definido no main.m.
    if isfield(simConfig, 'initialState') && ~isempty(simConfig.initialState)
        initialState = simConfig.initialState(:);

        if numel(initialState) ~= 12
            error('simConfig.initialState deve ser um vetor 12x1.');
        end

        state(:,1) = initialState;
    end

    rDesHist = zeros(3, N);
    vDesHist = zeros(3, N);
    aDesHist = zeros(3, N);
    psiDesHist = zeros(1, N);

    attitudeDesHist = zeros(3, N);
    motorOmegaHist = zeros(4, N);
    omegaSquaredHist = zeros(4, N);

    T_hist = zeros(1, N);
    tauHist = zeros(3, N);

    posErrorHist = zeros(3, N);
    attErrorHist = zeros(3, N);

    massHist = zeros(1, N);
    lapHist = zeros(1, N);
    windForceInertialHist = zeros(3, N);

    %% ---------------------- Estados internos dos controladores ----------
    posErrorInt = zeros(3,1);
    attErrorInt = zeros(3,1);

    %% ---------------------- Comandos iniciais ---------------------------
    T_cmd = GetQuadMass(quadCtrl)*quadCtrl.grav;

    attitudeDes = [
        0;
        0;
        traj.psi_des(1)
    ];

    tau_cmd = zeros(3,1);

    % Mixer usa modelo nominal. A planta real sera alterada na dinamica.
    [motorOmega, omegaSquared] = QuadrotorMixer(T_cmd, tau_cmd, quadCtrl);

    %% ---------------------- Loop principal ------------------------------
    for k = 1:N-1

        currentState = state(:,k);

        %% Condicoes de voo da volta atual
        [quadPlant, disturbance, lapIndex] = FlightConditionManager( ...
            quadNominal, ...
            t(k), ...
            traj, ...
            trajConfig, ...
            flightConfig);

        %% Referencia da trajetoria
        [r_des, v_des, a_des, psi_des] = GetTrajectoryReference(t(k), traj);

        %% Controlador de posicao
        if mod(k-1, N_pos) == 0
            [T_cmd, attitudeDes, posErrorInt, posInfo] = LocalPositionControlLoop( ...
                currentState, ...
                r_des, ...
                v_des, ...
                a_des, ...
                psi_des, ...
                controlConfig.position.type, ...
                posGains, ...
                positionUpdateFreq, ...
                quadCtrl, ...
                posErrorInt);
        end

        %% Controlador de atitude e mixer
        if mod(k-1, N_att) == 0
            [tau_cmd, attErrorInt, attInfo] = LocalAttitudeControlLoop( ...
                currentState, ...
                attitudeDes, ...
                controlConfig.attitude.type, ...
                attGains, ...
                attitudeUpdateFreq, ...
                attErrorInt);

            % Mixer nominal: transforma comandos desejados em velocidades
            % de motor com base no modelo conhecido pelo controlador.
            [motorOmega, omegaSquared] = QuadrotorMixer(T_cmd, tau_cmd, quadCtrl);
        end

        %% Integracao da dinamica
        switch method
            case "Euler"
                currentStateDot = EvaluateQuadrotorDynamics( ...
                    currentState, motorOmega, quadPlant, disturbance);
                nextState = currentState + Ts*currentStateDot;

            case "RK4"
                currentStateDot = EvaluateQuadrotorDynamics( ...
                    currentState, motorOmega, quadPlant, disturbance);
                nextState = RungeKutta4( ...
                    currentState, motorOmega, quadPlant, disturbance, Ts);

            otherwise
                error('Metodo de integracao nao implementado. Use "Euler" ou "RK4".');
        end

        %% Armazenamento
        stateDot(:,k) = currentStateDot;
        state(:,k+1) = nextState;

        rDesHist(:,k) = r_des;
        vDesHist(:,k) = v_des;
        aDesHist(:,k) = a_des;
        psiDesHist(k) = psi_des;

        attitudeDesHist(:,k) = attitudeDes;
        motorOmegaHist(:,k) = motorOmega;
        omegaSquaredHist(:,k) = omegaSquared;

        T_hist(k) = T_cmd;
        tauHist(:,k) = tau_cmd;

        posErrorHist(:,k) = r_des - currentState(1:3);
        attErrorHist(:,k) = attitudeDes - currentState(7:9);
        attErrorHist(3,k) = atan2(sin(attErrorHist(3,k)), cos(attErrorHist(3,k)));

        massHist(k) = GetQuadMass(quadPlant);
        lapHist(k) = lapIndex;
        windForceInertialHist(:,k) = disturbance.forceInertial;
    end

    %% Ultimo ponto
    [quadPlant, disturbance, lapIndex] = FlightConditionManager( ...
        quadNominal, ...
        t(N), ...
        traj, ...
        trajConfig, ...
        flightConfig);

    stateDot(:,N) = EvaluateQuadrotorDynamics( ...
        state(:,N), motorOmega, quadPlant, disturbance);

    [r_des, v_des, a_des, psi_des] = GetTrajectoryReference(t(N), traj);
    rDesHist(:,N) = r_des;
    vDesHist(:,N) = v_des;
    aDesHist(:,N) = a_des;
    psiDesHist(N) = psi_des;

    attitudeDesHist(:,N) = attitudeDes;
    motorOmegaHist(:,N) = motorOmega;
    omegaSquaredHist(:,N) = omegaSquared;

    T_hist(N) = T_cmd;
    tauHist(:,N) = tau_cmd;

    posErrorHist(:,N) = r_des - state(1:3,N);
    attErrorHist(:,N) = attitudeDes - state(7:9,N);
    attErrorHist(3,N) = atan2(sin(attErrorHist(3,N)), cos(attErrorHist(3,N)));

    massHist(N) = GetQuadMass(quadPlant);
    lapHist(N) = lapIndex;
    windForceInertialHist(:,N) = disturbance.forceInertial;

    %% ---------------------- Struct de saida -----------------------------
    sim = struct();

    sim.t = t;
    sim.state = state;
    sim.stateDot = stateDot;

    sim.r_des = rDesHist;
    sim.v_des = vDesHist;
    sim.a_des = aDesHist;
    sim.psi_des = psiDesHist;

    sim.attitude_des = attitudeDesHist;
    sim.motorOmega = motorOmegaHist;
    sim.omegaSquared = omegaSquaredHist;

    sim.T = T_hist;
    sim.tau = tauHist;

    sim.positionError = posErrorHist;
    sim.attitudeError = attErrorHist;

    % Mantido por compatibilidade: sim.quad aponta para o modelo nominal.
    sim.quad = quadNominal;
    sim.quadNominal = quadNominal;
    sim.quadCtrl = quadCtrl;
    sim.I_til = I_til;
    sim.traj = traj;

    % Novas saidas para analise das perturbacoes.
    sim.mass = massHist;
    sim.lapIndex = lapHist;
    sim.windForceInertial = windForceInertialHist;
    sim.windForce = windForceInertialHist;
    sim.flightConfig = flightConfig;

    sim.method = method;
    sim.Ts = Ts;
    sim.tf = tf;

    sim.positionUpdateFreq = positionUpdateFreq;
    sim.attitudeUpdateFreq = attitudeUpdateFreq;
    sim.N_pos = N_pos;
    sim.N_att = N_att;

    sim.controlConfig = controlConfig;
    sim.quadConfig = quadConfig;
    sim.trajConfig = trajConfig;
    sim.simConfig = simConfig;
    sim.gainInfo = gainInfo;

    %% ---------------------- Plot final ----------------------------------
    if plotData
        PlotControlledSimulation(sim);
    end
end

%% =======================================================================
function traj = ValidateTrajectoryStruct(traj)
% ValidateTrajectoryStruct
% -------------------------------------------------------------------------
% Garante que a trajetoria recebida pelo loop principal esta no formato
% esperado pelo interp1.
% -------------------------------------------------------------------------

    if ~isstruct(traj)
        error('traj deve ser uma struct.');
    end

    requiredFields = {'t', 'r_des', 'psi_des'};

    for i = 1:numel(requiredFields)
        fieldName = requiredFields{i};

        if ~isfield(traj, fieldName)
            error('traj precisa conter o campo %s.', fieldName);
        end
    end

    traj.t = traj.t(:);

    if size(traj.r_des, 2) ~= 3 && size(traj.r_des, 1) == 3
        traj.r_des = traj.r_des.';
    end

    if size(traj.r_des, 2) ~= 3
        error('traj.r_des deve ter dimensao N x 3.');
    end

    traj.psi_des = traj.psi_des(:);

    if numel(traj.t) ~= size(traj.r_des, 1)
        error('traj.t e traj.r_des possuem tamanhos incompatíveis.');
    end

    if numel(traj.t) ~= numel(traj.psi_des)
        error('traj.t e traj.psi_des possuem tamanhos incompatíveis.');
    end

    % Campos opcionais para feedforward. Se nao existirem, usa zero.
    if ~isfield(traj, 'v_des') || isempty(traj.v_des)
        traj.v_des = zeros(size(traj.r_des));
    end

    if ~isfield(traj, 'a_des') || isempty(traj.a_des)
        traj.a_des = zeros(size(traj.r_des));
    end

    if size(traj.v_des, 2) ~= 3 && size(traj.v_des, 1) == 3
        traj.v_des = traj.v_des.';
    end

    if size(traj.a_des, 2) ~= 3 && size(traj.a_des, 1) == 3
        traj.a_des = traj.a_des.';
    end

    if ~isequal(size(traj.v_des), size(traj.r_des))
        error('traj.v_des deve ter a mesma dimensao de traj.r_des.');
    end

    if ~isequal(size(traj.a_des), size(traj.r_des))
        error('traj.a_des deve ter a mesma dimensao de traj.r_des.');
    end

    if numel(traj.t) < 2
        error('traj.t precisa ter pelo menos duas amostras.');
    end

    if any(diff(traj.t) <= 0)
        error(['traj.t possui tempos repetidos ou fora de ordem. ', ...
               'Corrija a trajetoria antes de chamar ControlledMainLoop.']);
    end
end

%% =======================================================================
function [T, attitudeDes, errorIntOut, controlInfo] = LocalPositionControlLoop( ...
    state, r_des, v_des, a_des, psi_des, controlType, gains, updateFreq, quad, errorIntIn)
% LocalPositionControlLoop
% -------------------------------------------------------------------------
% Controle de posicao com feedforward da trajetoria.
%
% Antes:
%   u_pos = Kp*(r_d - r) + Kd*(0 - v)
%
% Agora:
%   u_pos = a_d + Kp*(r_d - r) + Kd*(v_d - v) + Ki*int(e_r)
%
% O termo a_d ajuda o quadrotor a antecipar a trajetoria e o termo v_d
% reduz o atraso de rastreamento.
% -------------------------------------------------------------------------

    TsController = 1/updateFreq;

    position = state(1:3);
    velocity = state(4:6);

    m = GetQuadMass(quad);
    g = quad.grav;

    r_des = r_des(:);
    v_des = v_des(:);
    a_des = a_des(:);

    posError = r_des - position;
    velError = v_des - velocity;

    switch string(controlType)
        case "P"
            errorIntOut = errorIntIn;
            u_feedback = PController(posError, gains.P);

        case "PD"
            errorIntOut = errorIntIn;
            u_feedback = PDController(posError, velError, gains.PD);

        case "PID"
            errorIntOut = errorIntIn + TsController*posError;
            u_feedback = PIDController(posError, velError, errorIntOut, gains.PID);

        otherwise
            error('Tipo de controlador de posicao nao implementado. Use "P", "PD" ou "PID".');
    end

    % Comando de aceleracao total no referencial inercial.
    u_pos = a_des + u_feedback;

    x_ddot_cmd = u_pos(1);
    y_ddot_cmd = u_pos(2);
    z_ddot_cmd = u_pos(3);

    T = m*(g + z_ddot_cmd);

    theta_des = (1/g)*(x_ddot_cmd*cos(psi_des) + y_ddot_cmd*sin(psi_des));
    phi_des   = (1/g)*(x_ddot_cmd*sin(psi_des) - y_ddot_cmd*cos(psi_des));

    %% Limite de inclinacao desejada
    maxTilt = deg2rad(20);
    phi_des = min(max(phi_des, -maxTilt), maxTilt);
    theta_des = min(max(theta_des, -maxTilt), maxTilt);

    attitudeDes = [
        phi_des;
        theta_des;
        psi_des
    ];

    controlInfo = struct();
    controlInfo.position = position;
    controlInfo.velocity = velocity;
    controlInfo.posError = posError;
    controlInfo.v_des = v_des;
    controlInfo.a_des = a_des;
    controlInfo.velError = velError;
    controlInfo.errorInt = errorIntOut;
    controlInfo.u_feedback = u_feedback;
    controlInfo.u_pos = u_pos;
    controlInfo.T = T;
    controlInfo.attitudeDes = attitudeDes;
end

%% =======================================================================
function [tau, errorIntOut, controlInfo] = LocalAttitudeControlLoop( ...
    state, attitudeDes, controlType, gains, updateFreq, errorIntIn)

    TsController = 1/updateFreq;

    attitude = state(7:9);
    angularRate = state(10:12);

    attError = attitudeDes - attitude;
    attError(3) = atan2(sin(attError(3)), cos(attError(3)));

    angularRateDes = zeros(3,1);
    rateError = angularRateDes - angularRate;

    switch string(controlType)
        case "P"
            errorIntOut = errorIntIn;
            tau = PController(attError, gains.P);

        case "PD"
            errorIntOut = errorIntIn;
            tau = PDController(attError, rateError, gains.PD);

        case "PID"
            errorIntOut = errorIntIn + TsController*attError;
            tau = PIDController(attError, rateError, errorIntOut, gains.PID);

        otherwise
            error('Tipo de controlador de atitude nao implementado. Use "P", "PD" ou "PID".');
    end

    controlInfo = struct();
    controlInfo.attitude = attitude;
    controlInfo.angularRate = angularRate;
    controlInfo.attitudeDes = attitudeDes;
    controlInfo.attitudeError = attError;
    controlInfo.rateError = rateError;
    controlInfo.errorInt = errorIntOut;
    controlInfo.tau = tau;
end

%% =======================================================================
function stateNext = RungeKutta4(state, motorOmega, quadPlant, disturbance, Ts)

    k1 = EvaluateQuadrotorDynamics(state, motorOmega, quadPlant, disturbance);
    k2 = EvaluateQuadrotorDynamics(state + 0.5*Ts*k1, motorOmega, quadPlant, disturbance);
    k3 = EvaluateQuadrotorDynamics(state + 0.5*Ts*k2, motorOmega, quadPlant, disturbance);
    k4 = EvaluateQuadrotorDynamics(state + Ts*k3, motorOmega, quadPlant, disturbance);

    stateNext = state + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end

%% =======================================================================
function stateDot = EvaluateQuadrotorDynamics(state, motorOmega, quadPlant, disturbance)
% Tenta chamar a nova QuadrotorDynamics com disturbance.
% Se a perturbacao for zero e a funcao antiga ainda estiver em uso, roda
% com a assinatura antiga. Se houver vento, exige a assinatura nova.

    try
        stateDot = QuadrotorDynamics(state, motorOmega, quadPlant, disturbance);
    catch ME
        if IsTooManyInputArgumentsError(ME)
            if IsZeroDisturbance(disturbance)
                stateDot = QuadrotorDynamics(state, motorOmega, quadPlant);
            else
                error(['QuadrotorDynamics.m ainda nao aceita disturbance. ', ...
                       'Para usar vento, altere a assinatura para ', ...
                       'QuadrotorDynamics(state, motorOmega, quad, disturbance) ', ...
                       'e some disturbance.forceInertial/m na aceleracao translacional.']);
            end
        else
            rethrow(ME);
        end
    end
end

%% =======================================================================
function [r_des, v_des, a_des, psi_des] = GetTrajectoryReference(tNow, traj)
% GetTrajectoryReference
% -------------------------------------------------------------------------
% Retorna posicao, velocidade, aceleracao e yaw desejados no instante tNow.
%
% Se traj.v_des ou traj.a_des nao existirem, usa zero como fallback. Isso
% mantem compatibilidade com trajetorias antigas.
% -------------------------------------------------------------------------

    if tNow <= traj.t(1)
        r_des = traj.r_des(1,:)';
        v_des = GetTrajectoryDerivativeSample(traj, 'v_des', 1);
        a_des = GetTrajectoryDerivativeSample(traj, 'a_des', 1);
        psi_des = traj.psi_des(1);
        return;
    end

    if tNow >= traj.t(end)
        r_des = traj.r_des(end,:)';
        v_des = GetTrajectoryDerivativeSample(traj, 'v_des', numel(traj.t));
        a_des = GetTrajectoryDerivativeSample(traj, 'a_des', numel(traj.t));
        psi_des = traj.psi_des(end);
        return;
    end

    x_des = interp1(traj.t, traj.r_des(:,1), tNow, 'linear');
    y_des = interp1(traj.t, traj.r_des(:,2), tNow, 'linear');
    z_des = interp1(traj.t, traj.r_des(:,3), tNow, 'linear');

    if isfield(traj, 'v_des') && ~isempty(traj.v_des)
        vx_des = interp1(traj.t, traj.v_des(:,1), tNow, 'linear');
        vy_des = interp1(traj.t, traj.v_des(:,2), tNow, 'linear');
        vz_des = interp1(traj.t, traj.v_des(:,3), tNow, 'linear');
    else
        vx_des = 0;
        vy_des = 0;
        vz_des = 0;
    end

    if isfield(traj, 'a_des') && ~isempty(traj.a_des)
        ax_des = interp1(traj.t, traj.a_des(:,1), tNow, 'linear');
        ay_des = interp1(traj.t, traj.a_des(:,2), tNow, 'linear');
        az_des = interp1(traj.t, traj.a_des(:,3), tNow, 'linear');
    else
        ax_des = 0;
        ay_des = 0;
        az_des = 0;
    end

    psi_des = interp1(traj.t, traj.psi_des, tNow, 'linear');

    r_des = [x_des; y_des; z_des];
    v_des = [vx_des; vy_des; vz_des];
    a_des = [ax_des; ay_des; az_des];
end

%% =======================================================================
function sample = GetTrajectoryDerivativeSample(traj, fieldName, idx)
% Retorna traj.(fieldName)(idx,:)' se existir; senao retorna zeros(3,1).

    if isfield(traj, fieldName) && ~isempty(traj.(fieldName))
        data = traj.(fieldName);

        if size(data, 2) ~= 3 && size(data, 1) == 3
            data = data.';
        end

        sample = data(idx,:)';
    else
        sample = zeros(3,1);
    end
end

%% =======================================================================
function flightConfig = DefaultFlightConfig(simConfig)

    if nargin < 1
        simConfig = struct();
    end

    flightConfig = struct();
    flightConfig.mode = "Nominal";

    if isstruct(simConfig) && isfield(simConfig, 'flightMode') && ~isempty(simConfig.flightMode)
        flightConfig.mode = string(simConfig.flightMode);
    end

    flightConfig.mass = struct();
    flightConfig.mass.enabled = false;
    flightConfig.mass.byLap = [];

    flightConfig.wind = struct();
    flightConfig.wind.enabled = false;
    flightConfig.wind.byLap = [];
    flightConfig.wind.referenceFrame = "inertial";
end

%% =======================================================================
function flightConfig = NormalizeFlightConfig(flightConfig)

    if ~isfield(flightConfig, 'mode') || isempty(flightConfig.mode)
        flightConfig.mode = "Nominal";
    else
        flightConfig.mode = string(flightConfig.mode);
    end

    if ~isfield(flightConfig, 'mass') || isempty(flightConfig.mass)
        flightConfig.mass = struct();
    end

    normalizedMode = NormalizeMode(flightConfig.mode);

    if ~isfield(flightConfig.mass, 'enabled') || isempty(flightConfig.mass.enabled)
        flightConfig.mass.enabled = normalizedMode == "masschange" || normalizedMode == "masswind";
    end

    if ~isfield(flightConfig.mass, 'byLap')
        flightConfig.mass.byLap = [];
    end

    if ~isfield(flightConfig, 'wind') || isempty(flightConfig.wind)
        flightConfig.wind = struct();
    end

    if ~isfield(flightConfig.wind, 'enabled') || isempty(flightConfig.wind.enabled)
        flightConfig.wind.enabled = normalizedMode == "wind" || normalizedMode == "masswind";
    end

    if ~isfield(flightConfig.wind, 'byLap')
        flightConfig.wind.byLap = [];
    end

    if ~isfield(flightConfig.wind, 'referenceFrame') || isempty(flightConfig.wind.referenceFrame)
        flightConfig.wind.referenceFrame = "inertial";
    else
        flightConfig.wind.referenceFrame = string(flightConfig.wind.referenceFrame);
    end
end

%% =======================================================================
function mode = NormalizeMode(modeIn)

    mode = lower(strtrim(string(modeIn)));
    mode = replace(mode, "_", "");
    mode = replace(mode, "-", "");
    mode = replace(mode, " ", "");

    switch mode
        case {"nominal", "modeloatual", "atual", "current", "normal"}
            mode = "nominal";

        case {"masschange", "massa", "mudancamassa", "mass", "payload"}
            mode = "masschange";

        case {"wind", "vento"}
            mode = "wind";

        case {"masswind", "massavento", "ventomassa", "massaevento", "windmass"}
            mode = "masswind";

        case {"trajectoryonly", "trajectory", "trajetoriaonly", "sotrajetoria", "trajetoria"}
            mode = "trajectoryonly";
    end
end

%% =======================================================================
function tf = IsTrajectoryOnlyMode(modeIn)

    tf = NormalizeMode(modeIn) == "trajectoryonly";
end

%% =======================================================================
function mass = GetQuadMass(quad)

    if isfield(quad, 'mass')
        mass = quad.mass;
    elseif isfield(quad, 'm')
        mass = quad.m;
    else
        error('A struct quad precisa ter o campo mass ou m.');
    end
end

%% =======================================================================
function quad = SetQuadMass(quad, mass)

    if ~isscalar(mass) || ~isnumeric(mass) || mass <= 0
        error('A massa da planta deve ser um escalar positivo.');
    end

    if isfield(quad, 'mass')
        quad.mass = mass;
    end

    if isfield(quad, 'm')
        quad.m = mass;
    end

    if ~isfield(quad, 'mass') && ~isfield(quad, 'm')
        quad.mass = mass;
    end
end

%% =======================================================================
function tf = IsScalarLogicalLike(x)

    tf = (islogical(x) || isnumeric(x)) && isscalar(x);
end

%% =======================================================================
function tf = IsZeroDisturbance(disturbance)

    tf = true;

    if isfield(disturbance, 'forceInertial')
        tf = tf && all(abs(disturbance.forceInertial(:)) < 1e-12);
    end

    if isfield(disturbance, 'forceBody')
        tf = tf && all(abs(disturbance.forceBody(:)) < 1e-12);
    end

    if isfield(disturbance, 'torqueBody')
        tf = tf && all(abs(disturbance.torqueBody(:)) < 1e-12);
    end
end

%% =======================================================================
function tf = IsTooManyInputArgumentsError(ME)

    msg = lower(string(ME.message));
    tf = contains(msg, "too many input arguments") || ...
         contains(msg, "muitos argumentos") || ...
         contains(msg, "argumentos de entrada em excesso");
end

%% =======================================================================
function sim = BuildTrajectoryOnlyOutput( ...
    traj, quadNominal, I_til, quadConfig, trajConfig, simConfig, ...
    controlConfig, flightConfig)
% Cria uma saida simples para o modo TrajectoryOnly.
% Nesse modo nao ha integracao dinamica, controlador, mixer ou perturbacao.

    t = traj.t(:).';
    N = numel(t);

    sim = struct();
    sim.t = t;
    sim.traj = traj;
    sim.r_des = traj.r_des.';

    if isfield(traj, 'v_des') && ~isempty(traj.v_des)
        sim.v_des = traj.v_des.';
    else
        sim.v_des = zeros(3, N);
    end

    if isfield(traj, 'a_des') && ~isempty(traj.a_des)
        sim.a_des = traj.a_des.';
    else
        sim.a_des = zeros(3, N);
    end

    sim.psi_des = traj.psi_des(:).';

    sim.state = zeros(12, N);
    sim.state(1:3,:) = sim.r_des;
    sim.state(9,:) = sim.psi_des;
    sim.stateDot = zeros(12, N);

    sim.attitude_des = zeros(3, N);
    sim.attitude_des(3,:) = sim.psi_des;

    sim.motorOmega = zeros(4, N);
    sim.omegaSquared = zeros(4, N);
    sim.T = zeros(1, N);
    sim.tau = zeros(3, N);
    sim.positionError = zeros(3, N);
    sim.attitudeError = zeros(3, N);

    sim.quad = quadNominal;
    sim.quadNominal = quadNominal;
    sim.quadCtrl = quadNominal;
    sim.I_til = I_til;

    sim.mass = GetQuadMass(quadNominal)*ones(1, N);
    sim.lapIndex = ones(1, N);
    sim.windForceInertial = zeros(3, N);
    sim.windForce = sim.windForceInertial;

    sim.flightConfig = flightConfig;
    sim.method = "TrajectoryOnly";
    sim.Ts = simConfig.Ts;
    sim.tf = t(end);

    sim.positionUpdateFreq = NaN;
    sim.attitudeUpdateFreq = NaN;
    sim.N_pos = NaN;
    sim.N_att = NaN;

    sim.controlConfig = controlConfig;
    sim.quadConfig = quadConfig;
    sim.trajConfig = trajConfig;
    sim.simConfig = simConfig;
end

%% =======================================================================
function PlotTrajectoryOnlySimulation(sim)

    figure('Name', 'Trajectory Only', ...
           'Position', [100 100 1000 600]);

    plot3(sim.r_des(1,:), sim.r_des(2,:), sim.r_des(3,:), ...
          '--', 'LineWidth', 1.8);
    grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Trajetoria planejada - modo TrajectoryOnly');
    view(35, 25);
end

%% =======================================================================
function PlotControlledSimulation(sim)

    t = sim.t;
    state = sim.state;

    x = state(1,:);
    y = state(2,:);
    z = state(3,:);

    x_dot = state(4,:);
    y_dot = state(5,:);
    z_dot = state(6,:);

    phi = state(7,:);
    theta = state(8,:);
    psi = state(9,:);

    figure('Name', 'Controlled Quadrotor Simulation', ...
           'Position', [100 100 1400 700]);

    tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    %% ===================================================================
    % Trajetoria 3D com cor diferente por volta
    % ====================================================================

    nexttile(1, [3 2]);
    hold on;

    if isfield(sim, 'lapIndex') && ~isempty(sim.lapIndex)

        lapVec = sim.lapIndex(:).';
        validLap = isfinite(lapVec);
        uniqueLaps = unique(lapVec(validLap));

        nLaps = numel(uniqueLaps);
        lapColors = lines(max(nLaps, 1));

        for iLap = 1:nLaps

            lapNumber = uniqueLaps(iLap);
            idx = lapVec == lapNumber;

            plot3(x(idx), y(idx), z(idx), ...
                'LineWidth', 2.0, ...
                'Color', lapColors(iLap,:), ...
                'DisplayName', sprintf('Quadrirotor - volta %d', lapNumber));

            plot3(sim.r_des(1,idx), sim.r_des(2,idx), sim.r_des(3,idx), ...
                '--', ...
                'LineWidth', 1.4, ...
                'Color', lapColors(iLap,:), ...
                'HandleVisibility', 'off');
        end

        plot3(nan, nan, nan, '--', ...
            'Color', [0.8 0.8 0.8], ...
            'LineWidth', 1.4, ...
            'DisplayName', 'Referencia');

    else

        plot3(x, y, z, ...
            'LineWidth', 1.8, ...
            'DisplayName', 'Quadrirotor');

        plot3(sim.r_des(1,:), sim.r_des(2,:), sim.r_des(3,:), ...
            '--', ...
            'LineWidth', 1.4, ...
            'DisplayName', 'Referencia');
    end

    grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Trajetoria 3D');
    legend('Location', 'best');
    view(35, 25);

    %% Ajuste automatico dos limites do grafico 3D

    allX = [x, sim.r_des(1,:)];
    allY = [y, sim.r_des(2,:)];
    allZ = [z, sim.r_des(3,:)];

    marginX = 0.10*max(1, range(allX));
    marginY = 0.10*max(1, range(allY));
    marginZ = 0.10*max(1, range(allZ));

    xlim([min(allX)-marginX, max(allX)+marginX]);
    ylim([min(allY)-marginY, max(allY)+marginY]);
    zlim([min(allZ)-marginZ, max(allZ)+marginZ]);

    %% ===================================================================
    % Posicoes
    % ====================================================================

    nexttile(3);

    plot(t, x, 'LineWidth', 1.2);
    hold on;
    plot(t, y, 'LineWidth', 1.2);
    plot(t, z, 'LineWidth', 1.2);

    plot(t, sim.r_des(1,:), '--', 'LineWidth', 1.0);
    plot(t, sim.r_des(2,:), '--', 'LineWidth', 1.0);
    plot(t, sim.r_des(3,:), '--', 'LineWidth', 1.0);

    grid on;
    xlabel('Tempo [s]');
    ylabel('Posicao [m]');
    title('Posicao');
    legend('x', 'y', 'z', 'x_d', 'y_d', 'z_d', 'Location', 'best');

    %% ===================================================================
    % Velocidades lineares
    % ====================================================================

    nexttile(6);

    plot(t, x_dot, 'LineWidth', 1.2);
    hold on;
    plot(t, y_dot, 'LineWidth', 1.2);
    plot(t, z_dot, 'LineWidth', 1.2);

    grid on;
    xlabel('Tempo [s]');
    ylabel('Velocidade [m/s]');
    title('Velocidades lineares');
    legend('x dot', 'y dot', 'z dot', 'Location', 'best');

    %% ===================================================================
    % Atitude
    % ====================================================================

    nexttile(9);

    plot(t, rad2deg(phi), 'LineWidth', 1.2);
    hold on;
    plot(t, rad2deg(theta), 'LineWidth', 1.2);
    plot(t, rad2deg(psi), 'LineWidth', 1.2);

    plot(t, rad2deg(sim.attitude_des(1,:)), '--', 'LineWidth', 1.0);
    plot(t, rad2deg(sim.attitude_des(2,:)), '--', 'LineWidth', 1.0);
    plot(t, rad2deg(sim.attitude_des(3,:)), '--', 'LineWidth', 1.0);

    grid on;
    xlabel('Tempo [s]');
    ylabel('Angulo [graus]');
    title('Atitude');
    legend('\phi', '\theta', '\psi', '\phi_d', '\theta_d', '\psi_d', 'Location', 'best');

    %% ===================================================================
    % Figura adicional para condicoes de voo
    % ====================================================================

    if isfield(sim, 'mass') && isfield(sim, 'windForceInertial')

        figure('Name', 'Condicoes de voo', ...
               'Position', [150 150 1100 500]);

        tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

        nexttile;
        stairs(t, sim.lapIndex, 'LineWidth', 1.4);
        grid on;
        xlabel('Tempo [s]');
        ylabel('Volta');
        title('Indice da volta');

        nexttile;
        plot(t, sim.mass, 'LineWidth', 1.4);
        grid on;
        xlabel('Tempo [s]');
        ylabel('Massa [kg]');
        title('Massa real usada na planta');

        nexttile;
        plot(t, sim.windForceInertial(1,:), 'LineWidth', 1.2);
        hold on;
        plot(t, sim.windForceInertial(2,:), 'LineWidth', 1.2);
        plot(t, sim.windForceInertial(3,:), 'LineWidth', 1.2);

        grid on;
        xlabel('Tempo [s]');
        ylabel('Forca [N]');
        title('Forca de vento no referencial inercial');
        legend('F_x', 'F_y', 'F_z', 'Location', 'best');
    end

    %% ===================================================================
    % Velocidade angular dos motores
    % ====================================================================

    motorOmega = GetMotorOmegaForPlot(sim);

    if ~isempty(motorOmega)

        figure('Name', 'Velocidade angular dos motores', ...
               'Position', [200 200 1100 500]);

        plot(t, motorOmega(1,:), 'LineWidth', 1.3);
        hold on;
        plot(t, motorOmega(2,:), 'LineWidth', 1.3);
        plot(t, motorOmega(3,:), 'LineWidth', 1.3);
        plot(t, motorOmega(4,:), 'LineWidth', 1.3);

        grid on;
        xlabel('Tempo [s]');
        ylabel('\omega_i [rad/s]');
        title('Velocidade angular dos motores');
        legend('\omega_1', '\omega_2', '\omega_3', '\omega_4', ...
               'Location', 'best');

    else

        warning('Nao foi possivel plotar omega dos motores. sim.motorOmega ou sim.omegaSquared nao foi encontrado.');
    end

    %% ===================================================================
    % Erro de atitude
    % ====================================================================

    PlotTrackingErrors(sim);
end

%% =======================================================================
function PlotTrackingErrors(sim)

    t = sim.t;

    if ~isfield(sim, 'positionError') || isempty(sim.positionError)
        warning('Nao foi possivel plotar erros de posicao. Campo sim.positionError nao encontrado.');
        return;
    end

    if ~isfield(sim, 'attitudeError') || isempty(sim.attitudeError)
        warning('Nao foi possivel plotar erros de atitude. Campo sim.attitudeError nao encontrado.');
        return;
    end

    posError = sim.positionError;
    attError = sim.attitudeError;

    if size(posError, 1) ~= 3 && size(posError, 2) == 3
        posError = posError.';
    end

    if size(attError, 1) ~= 3 && size(attError, 2) == 3
        attError = attError.';
    end

    if size(posError, 1) ~= 3
        warning('sim.positionError deve ter dimensao 3xN ou Nx3.');
        return;
    end

    if size(attError, 1) ~= 3
        warning('sim.attitudeError deve ter dimensao 3xN ou Nx3.');
        return;
    end

    N = numel(t);

    if size(posError, 2) < N
        posError = [posError, repmat(posError(:,end), 1, N - size(posError,2))];
    end

    if size(attError, 2) < N
        attError = [attError, repmat(attError(:,end), 1, N - size(attError,2))];
    end

    posError = posError(:, 1:N);
    attError = attError(:, 1:N);

    figure('Name', 'Erros de rastreamento', ...
           'Position', [250 250 1150 650]);

    tiledlayout(2, 1, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact');

    %% Erro de posicao
    nexttile;

    plot(t, posError(1,:), 'LineWidth', 1.3);
    hold on;
    plot(t, posError(2,:), 'LineWidth', 1.3);
    plot(t, posError(3,:), 'LineWidth', 1.3);

    yline(0, '--', 'LineWidth', 1.0);

    grid on;
    xlabel('Tempo [s]');
    ylabel('Erro de posicao [m]');
    title('Erro de posicao: e_r = r_d - r');
    legend('e_x', 'e_y', 'e_z', 'Referencia zero', 'Location', 'best');

    %% Erro de atitude
    nexttile;

    plot(t, rad2deg(attError(1,:)), 'LineWidth', 1.3);
    hold on;
    plot(t, rad2deg(attError(2,:)), 'LineWidth', 1.3);
    plot(t, rad2deg(attError(3,:)), 'LineWidth', 1.3);

    yline(0, '--', 'LineWidth', 1.0);

    grid on;
    xlabel('Tempo [s]');
    ylabel('Erro de atitude [graus]');
    title('Erro de atitude: e_\eta = \eta_d - \eta');
    legend('e_\phi', 'e_\theta', 'e_\psi', 'Referencia zero', 'Location', 'best');
end

%% =======================================================================
function motorOmega = GetMotorOmegaForPlot(sim)

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

        warning('motorOmega deve ter dimensao 4xN ou Nx4.');
        motorOmega = [];
        return;
    end

    N = numel(sim.t);

    if size(motorOmega, 2) < N
        motorOmega = [motorOmega, repmat(motorOmega(:,end), 1, N - size(motorOmega,2))];
    end

    motorOmega = motorOmega(:, 1:N);
end
