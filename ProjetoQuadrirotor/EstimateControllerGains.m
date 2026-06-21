function [posGains, attGains, gainInfo] = EstimateControllerGainsByTime(quad, gainSpec, settlingFactor, pidGamma)
% EstimateControllerGainsByTime
% -------------------------------------------------------------------------
% Calcula ganhos PD e PID a partir de tempo de pico e tempo de estabilizacao.
%
% Entrada:
%   gainSpec = [tp_pos, ts_pos, tp_att, ts_att]
%
% onde:
%   tp_pos -> tempo de pico usado igualmente para x, y, z
%   ts_pos -> tempo de estabilizacao usado igualmente para x, y, z
%   tp_att -> tempo de pico usado igualmente para phi, theta, psi
%   ts_att -> tempo de estabilizacao usado igualmente para phi, theta, psi
%
% Saida:
%   posGains.P, posGains.PD, posGains.PID
%   attGains.P, attGains.PD, attGains.PID
%
% Observacao:
%   Para posicao, os ganhos sao usados diretamente, pois o controlador de
%   posicao gera uma aceleracao desejada.
%
%   Para atitude, os ganhos sao multiplicados pelas inercias Ixx, Iyy e Izz,
%   pois o controlador de atitude gera torque.
% -------------------------------------------------------------------------

    if nargin < 3 || isempty(settlingFactor)
        settlingFactor = 4.0;
    end

    if nargin < 4 || isempty(pidGamma)
        pidGamma = 5;
    end

    gainSpec = gainSpec(:).';

    if numel(gainSpec) ~= 4
        error('gainSpec deve ter 4 valores: [tp_pos, ts_pos, tp_att, ts_att].');
    end

    tp_pos = gainSpec(1);
    ts_pos = gainSpec(2);
    tp_att = gainSpec(3);
    ts_att = gainSpec(4);

    beta = settlingFactor;
    gamma = pidGamma;

    %% ---------------------- Ganhos de posicao ---------------------------
    pd_pos = SecondOrderTimeGains(tp_pos, ts_pos, beta, gamma, "PD");
    pid_pos = SecondOrderTimeGains(tp_pos, ts_pos, beta, gamma, "PID");

    posGains = struct();

    % P e mantido por compatibilidade. Ele usa apenas o Kp do projeto PD.
    posGains.P.Kp = diag([
        pd_pos.Kp;
        pd_pos.Kp;
        pd_pos.Kp
    ]);

    % PD para x, y, z.
    posGains.PD.Kp = diag([
        pd_pos.Kp;
        pd_pos.Kp;
        pd_pos.Kp
    ]);

    posGains.PD.Kd = diag([
        pd_pos.Kd;
        pd_pos.Kd;
        pd_pos.Kd
    ]);

    % PID para x, y, z.
    posGains.PID.Kp = diag([
        pid_pos.Kp;
        pid_pos.Kp;
        pid_pos.Kp
    ]);

    posGains.PID.Kd = diag([
        pid_pos.Kd;
        pid_pos.Kd;
        pid_pos.Kd
    ]);

    posGains.PID.Ki = diag([
        pid_pos.Ki;
        pid_pos.Ki;
        pid_pos.Ki
    ]);

    %% ---------------------- Ganhos de atitude ---------------------------
    pd_att_base = SecondOrderTimeGains(tp_att, ts_att, beta, gamma, "PD");
    pid_att_base = SecondOrderTimeGains(tp_att, ts_att, beta, gamma, "PID");

    I = GetInertiaMatrix(quad);

    Ixx = I(1,1);
    Iyy = I(2,2);
    Izz = I(3,3);

    attGains = struct();

    % P por compatibilidade.
    attGains.P.Kp = diag([
        Ixx*pd_att_base.Kp;
        Iyy*pd_att_base.Kp;
        Izz*pd_att_base.Kp
    ]);

    % PD para phi, theta, psi.
    attGains.PD.Kp = diag([
        Ixx*pd_att_base.Kp;
        Iyy*pd_att_base.Kp;
        Izz*pd_att_base.Kp
    ]);

    attGains.PD.Kd = diag([
        Ixx*pd_att_base.Kd;
        Iyy*pd_att_base.Kd;
        Izz*pd_att_base.Kd
    ]);

    % PID para phi, theta, psi.
    attGains.PID.Kp = diag([
        Ixx*pid_att_base.Kp;
        Iyy*pid_att_base.Kp;
        Izz*pid_att_base.Kp
    ]);

    attGains.PID.Kd = diag([
        Ixx*pid_att_base.Kd;
        Iyy*pid_att_base.Kd;
        Izz*pid_att_base.Kd
    ]);

    attGains.PID.Ki = diag([
        Ixx*pid_att_base.Ki;
        Iyy*pid_att_base.Ki;
        Izz*pid_att_base.Ki
    ]);

    %% ---------------------- Informacoes de saida ------------------------
    gainInfo = struct();

    gainInfo.gainSpec = gainSpec;
    gainInfo.settlingFactor = beta;
    gainInfo.pidGamma = gamma;

    gainInfo.position.tp = tp_pos;
    gainInfo.position.ts = ts_pos;
    gainInfo.position.PD = pd_pos;
    gainInfo.position.PID = pid_pos;

    gainInfo.attitude.tp = tp_att;
    gainInfo.attitude.ts = ts_att;
    gainInfo.attitude.PD_base = pd_att_base;
    gainInfo.attitude.PID_base = pid_att_base;
    gainInfo.attitude.inertia = [Ixx; Iyy; Izz];

    gainInfo.position.gains = posGains;
    gainInfo.attitude.gains = attGains;
end

%% ========================================================================
function gains = SecondOrderTimeGains(tp, ts, beta, gamma, controllerType)

    if ~isnumeric(tp) || ~isscalar(tp) || tp <= 0
        error('tp deve ser positivo.');
    end

    if ~isnumeric(ts) || ~isscalar(ts) || ts <= 0
        error('ts deve ser positivo.');
    end

    if ~isnumeric(beta) || ~isscalar(beta) || beta <= 0
        error('settlingFactor deve ser positivo.');
    end

    if ~isnumeric(gamma) || ~isscalar(gamma) || gamma <= 0
        error('pidGamma deve ser positivo.');
    end

    sigma = beta/ts;
    wd = pi/tp;

    wn = sqrt(sigma^2 + wd^2);
    zeta = sigma/wn;

    controllerType = upper(string(controllerType));

    switch controllerType

        case "PD"

            Kp = wn^2;
            Kd = 2*sigma;
            Ki = 0;
            p3 = NaN;

        case "PID"

            % Polo adicional associado ao integrador.
            p3 = gamma*sigma;

            % Polinomio desejado:
            %   (s^2 + 2*sigma*s + wn^2)*(s + p3)
            % Logo:
            %   s^3 + Kd*s^2 + Kp*s + Ki
            Kd = 2*sigma + p3;
            Kp = wn^2 + 2*sigma*p3;
            Ki = wn^2*p3;

        otherwise

            error('controllerType deve ser "PD" ou "PID".');
    end

    gains = struct();

    gains.tp = tp;
    gains.ts = ts;
    gains.sigma = sigma;
    gains.wd = wd;
    gains.wn = wn;
    gains.zeta = zeta;
    gains.p3 = p3;

    gains.Kp = Kp;
    gains.Kd = Kd;
    gains.Ki = Ki;
end

%% ========================================================================
function I = GetInertiaMatrix(quad)

    if isfield(quad, 'Inertia') && ~isempty(quad.Inertia)

        I = quad.Inertia;

    elseif isfield(quad, 'I') && ~isempty(quad.I)

        I = quad.I;

    elseif isfield(quad, 'J') && ~isempty(quad.J)

        I = quad.J;

    else

        error('Nao foi encontrado tensor de inercia em quad.Inertia, quad.I ou quad.J.');
    end

    if isvector(I) && numel(I) == 3
        I = diag(I(:));
    end

    if ~isequal(size(I), [3 3])
        error('O tensor de inercia deve ser 3x3 ou vetor com 3 elementos.');
    end
end
