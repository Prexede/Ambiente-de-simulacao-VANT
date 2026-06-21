function stateDot = QuadrotorDynamics(state, motorOmega, quad, disturbance)
% QuadrotorDynamics
% -------------------------------------------------------------------------
% Modelo dinamico nao linear simplificado de um quadrirotor.
%
% Esta versao foi alterada para permitir perturbacoes externas, mantendo
% compatibilidade com a chamada antiga:
%
%   stateDot = QuadrotorDynamics(state, motorOmega, quad)
%
% e com a nova chamada usada no ControlledMainLoop:
%
%   stateDot = QuadrotorDynamics(state, motorOmega, quadPlant, disturbance)
%
% ENTRADAS:
%   state : vetor de estados 12x1
%
%       state = [
%           x;
%           y;
%           z;
%           x_dot;
%           y_dot;
%           z_dot;
%           phi;
%           theta;
%           psi;
%           p;
%           q;
%           r
%       ];
%
%   motorOmega : velocidades angulares dos motores [rad/s]
%
%       motorOmega = [
%           omega_1;
%           omega_2;
%           omega_3;
%           omega_4
%       ];
%
%   quad : struct gerada por EstimatedQuadParameters ou modificada pelo
%          gerenciador de condicoes de voo.
%
%          Campos usados nesta funcao:
%             quad.mass      -> massa real da planta [kg]
%             quad.Inertia   -> matriz de inercia 3x3 [kg.m^2]
%             quad.armLength -> comprimento efetivo do braco [m]
%             quad.grav      -> gravidade [m/s^2]
%             quad.k_aero    -> coeficiente de empuxo [N/(rad/s)^2]
%             quad.k_drag    -> coeficiente de torque de arrasto
%
%   disturbance : struct opcional com perturbacoes externas.
%
%       Campos principais:
%           disturbance.forceInertial -> forca externa no referencial
%                                        inercial [Fx; Fy; Fz] [N]
%
%           disturbance.forceBody     -> forca externa no referencial do
%                                        corpo [Fx; Fy; Fz] [N]
%
%           disturbance.torqueBody    -> torque externo no referencial do
%                                        corpo [Tx; Ty; Tz] [N.m]
%
%       Para o caso do vento definido por volta, o ControlledMainLoop deve
%       preencher disturbance.forceInertial.
%
% SAIDA:
%   stateDot : derivada temporal do vetor de estados 12x1
%
%       stateDot = [
%           x_dot;
%           y_dot;
%           z_dot;
%           x_ddot;
%           y_ddot;
%           z_ddot;
%           phi_dot;
%           theta_dot;
%           psi_dot;
%           p_dot;
%           q_dot;
%           r_dot
%       ];
%
% CONVENCAO:
%   - z positivo para cima;
%   - gravidade atua em -z;
%   - empuxo atua no eixo z do corpo;
%   - disturbance.forceInertial ja esta no referencial inercial;
%   - disturbance.forceBody e convertida para o referencial inercial usando R.
%
% -------------------------------------------------------------------------

    %% ---------------------- Tratamento minimo de entrada ----------------
    if nargin < 4 || isempty(disturbance)
        disturbance = DefaultDisturbance();
    else
        disturbance = CompleteDisturbance(disturbance);
    end

    state = state(:);
    motorOmega = motorOmega(:);

    if numel(state) ~= 12
        error('state deve ser um vetor 12x1.');
    end

    if numel(motorOmega) ~= 4
        error('motorOmega deve ser um vetor 4x1.');
    end

    %% ---------------------- Estados -------------------------------------
    x_dot = state(4);
    y_dot = state(5);
    z_dot = state(6);

    phi   = state(7);
    theta = state(8);
    psi   = state(9);

    p     = state(10);
    q     = state(11);
    r     = state(12);

    %% ---------------------- Parametros do quadrirotor -------------------
    m      = quad.mass;
    I      = quad.Inertia;
    l      = quad.armLength;
    g      = quad.grav;
    k_aero = quad.k_aero;
    k_drag = quad.k_drag;

    if m <= 0
        error('quad.mass deve ser positivo.');
    end

    %% ---------------------- Velocidade dos motores ----------------------
    omega_1 = motorOmega(1);
    omega_2 = motorOmega(2);
    omega_3 = motorOmega(3);
    omega_4 = motorOmega(4);

    %% ---------------------- Empuxos individuais -------------------------
    T1 = k_aero*omega_1^2;
    T2 = k_aero*omega_2^2;
    T3 = k_aero*omega_3^2;
    T4 = k_aero*omega_4^2;

    %% ---------------------- Empuxo total e torques ----------------------
    T = T1 + T2 + T3 + T4;

    tau_x = l*(T4 - T2);
    tau_y = l*(T1 - T3);
    tau_z = k_drag*(-omega_1^2 + omega_2^2 - omega_3^2 + omega_4^2);

    tauMotor = [
        tau_x;
        tau_y;
        tau_z
    ];

    %% ---------------------- Matriz de rotacao ZYX -----------------------
    R = RotationMatrixZYX(phi, theta, psi);

    %% ---------------------- Dinamica translacional ----------------------
    % Forcas consideradas:
    %   1) empuxo dos motores no referencial do corpo, convertido para o
    %      referencial inercial;
    %   2) peso no referencial inercial;
    %   3) forca externa diretamente no referencial inercial;
    %   4) forca externa no corpo convertida para o referencial inercial.

    posDot = [
        x_dot;
        y_dot;
        z_dot
    ];

    F_thrust_inertial = R*[0; 0; T];
    F_gravity_inertial = [0; 0; -m*g];

    F_external_inertial = disturbance.forceInertial;
    F_external_body_as_inertial = R*disturbance.forceBody;

    F_total_inertial = F_thrust_inertial + ...
                       F_gravity_inertial + ...
                       F_external_inertial + ...
                       F_external_body_as_inertial;

    acc = F_total_inertial/m;

    %% ---------------------- Cinematica rotacional -----------------------
    omegaBody = [
        p;
        q;
        r
    ];

    etaDot = EulerRatesZYX(phi, theta, omegaBody);

    %% ---------------------- Dinamica rotacional -------------------------
    % Modelo simplificado:
    %   I*w_dot = tau - w x (I*w)
    %
    % Agora tambem e possivel somar torque externo no corpo:
    %   tauTotal = tauMotor + disturbance.torqueBody

    tauTotal = tauMotor + disturbance.torqueBody;
    omegaDot = I\(tauTotal - cross(omegaBody, I*omegaBody));

    %% ---------------------- Derivada dos estados ------------------------
    stateDot = [
        posDot;
        acc;
        etaDot;
        omegaDot
    ];

end

%% =======================================================================
function disturbance = DefaultDisturbance()
% Cria uma perturbacao nula padrao.

    disturbance = struct();
    disturbance.forceInertial = zeros(3,1);
    disturbance.forceBody = zeros(3,1);
    disturbance.torqueBody = zeros(3,1);
end

%% =======================================================================
function disturbance = CompleteDisturbance(disturbance)
% Completa campos ausentes da struct disturbance.
%
% Esta funcao tambem aceita alguns nomes alternativos para facilitar testes.
% O nome recomendado para o vento por volta e forceInertial.

    if ~isstruct(disturbance)
        error('disturbance deve ser uma struct.');
    end

    forceInertial = ReadVector3Field( ...
        disturbance, ...
        {'forceInertial', 'windForceInertial', 'F_inertial', 'forceWorld'}, ...
        zeros(3,1));

    forceBody = ReadVector3Field( ...
        disturbance, ...
        {'forceBody', 'windForceBody', 'F_body'}, ...
        zeros(3,1));

    torqueBody = ReadVector3Field( ...
        disturbance, ...
        {'torqueBody', 'tauBody', 'externalTorqueBody'}, ...
        zeros(3,1));

    disturbance.forceInertial = forceInertial;
    disturbance.forceBody = forceBody;
    disturbance.torqueBody = torqueBody;
end

%% =======================================================================
function value = ReadVector3Field(s, possibleNames, defaultValue)
% Le o primeiro campo existente entre possibleNames e retorna vetor coluna 3x1.

    value = defaultValue;

    for i = 1:numel(possibleNames)
        fieldName = possibleNames{i};

        if isfield(s, fieldName) && ~isempty(s.(fieldName))
            rawValue = s.(fieldName);

            if ~isnumeric(rawValue) || numel(rawValue) ~= 3
                error('O campo disturbance.%s deve ser um vetor numerico com 3 elementos.', fieldName);
            end

            value = rawValue(:);
            return;
        end
    end
end

%% =======================================================================
function R = RotationMatrixZYX(phi, theta, psi)
% Matriz de rotacao corpo -> inercial usando sequencia ZYX:
%
%   R = Rz(psi)*Ry(theta)*Rx(phi)

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
function etaDot = EulerRatesZYX(phi, theta, omegaBody)
% Converte velocidades angulares do corpo [p q r] em derivadas dos angulos
% de Euler [phi_dot theta_dot psi_dot], usando sequencia ZYX.

    p = omegaBody(1);
    q = omegaBody(2);
    r = omegaBody(3);

    etaDot = [
        1, sin(phi)*tan(theta), cos(phi)*tan(theta);
        0, cos(phi),           -sin(phi);
        0, sin(phi)/cos(theta), cos(phi)/cos(theta)
    ] * [
        p;
        q;
        r
    ];
end
