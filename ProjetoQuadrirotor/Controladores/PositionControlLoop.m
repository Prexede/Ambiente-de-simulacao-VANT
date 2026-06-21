function [T, attitudeDes, errorIntOut, controlInfo] = PositionControlLoop(state, r_des, psi_des, controlType, gains, updateFreq, quad, errorIntIn)
% PositionControlLoop
% -------------------------------------------------------------------------
% Controlador de posicao do quadrirotor.
%
% Calcula:
%   - empuxo total desejado T;
%   - atitude desejada [phi_des; theta_des; psi_des].
%
% ENTRADAS:
%   state       : vetor de estados 12x1
%   r_des       : posicao desejada 3x1
%   psi_des     : yaw desejado [rad]
%   controlType : "P", "PD" ou "PID"
%   gains       : struct gerada por PositionPIDGains()
%   updateFreq  : frequencia de atualizacao do controlador [Hz]
%   quad        : struct gerada por EstimatedQuadParameters
%
% ENTRADA OPCIONAL:
%   errorIntIn  : matriz 3x1 com a integral do erro de posicao
%
% SAIDAS:
%   T           : empuxo total desejado [N]
%   attitudeDes : [phi_des; theta_des; psi_des]
%   errorIntOut : matriz 3x1 com a integral atualizada
%   controlInfo : struct auxiliar
%
% -------------------------------------------------------------------------

    %% ---------------------- Tratamento minimo de entrada ----------------
    if nargin < 1 || isempty(state)
        error('Informe o vetor de estados state.');
    end

    if nargin < 2 || isempty(r_des)
        error('Informe a posicao desejada r_des.');
    end

    if nargin < 3 || isempty(psi_des)
        error('Informe psi_des.');
    end

    if nargin < 4 || isempty(controlType)
        error('Informe controlType: "P", "PD" ou "PID".');
    end

    if nargin < 5 || isempty(gains)
        error('Informe os ganhos gerados por PositionPIDGains().');
    end

    if nargin < 6 || isempty(updateFreq)
        error('Informe updateFreq em Hz.');
    end

    if nargin < 7 || isempty(quad)
        error('Informe a struct quad.');
    end

    %% ---------------------- Entrada opcional ----------------------------
    if nargin < 8 || isempty(errorIntIn)
        errorIntIn = zeros(3,1);
    end

    %% ---------------------- Periodo equivalente -------------------------
    TsController = 1/updateFreq;

    %% ---------------------- Estados de posicao --------------------------
    position = [
        state(1);
        state(2);
        state(3)
    ];

    velocity = [
        state(4);
        state(5);
        state(6)
    ];

    %% ---------------------- Parametros ----------------------------------
    m = quad.mass;
    g = quad.grav;

    %% ---------------------- Erros de posicao ----------------------------
    posError = r_des - position;

    % Como o planejador atual retorna apenas posicao desejada,
    % assume-se velocidade desejada igual a zero.
    velDes = zeros(3,1);
    velError = velDes - velocity;

    %% ---------------------- Controlador generico ------------------------
    controlType = string(controlType);

    switch controlType
        case "P"
            errorIntOut = errorIntIn;
            u_pos = PController(posError, gains.P);

        case "PD"
            errorIntOut = errorIntIn;
            u_pos = PDController(posError, velError, gains.PD);

        case "PID"
            errorIntOut = errorIntIn + TsController*posError;
            u_pos = PIDController(posError, velError, errorIntOut, gains.PID);

        otherwise
            error('controlType nao implementado. Use "P", "PD" ou "PID".');
    end

    %% ---------------------- Comando de aceleracao -----------------------
    x_ddot_cmd = u_pos(1);
    y_ddot_cmd = u_pos(2);
    z_ddot_cmd = u_pos(3);

    %% ---------------------- Empuxo total --------------------------------
    % Convencao:
    %   z positivo para cima
    %   acc_z = T/m - g
    %
    % Logo:
    %   T = m*(g + z_ddot_cmd)

    T = m*(g + z_ddot_cmd);

    %% ---------------------- Atitude desejada ----------------------------
    % Linearizacao em torno do hover:
    %
    %   x_ddot = -g*theta*cos(psi) + g*phi*sin(psi)
    %   y_ddot = -g*theta*sin(psi) - g*phi*cos(psi)
    %
    % Rearranjando:
    %
    %   theta_des = -(1/g)*(x_ddot*cos(psi) + y_ddot*sin(psi))
    %   phi_des   = -(1/g)*(x_ddot*sin(psi) - y_ddot*cos(psi))

    theta_des = -(1/g)*(x_ddot_cmd*cos(psi_des) + y_ddot_cmd*sin(psi_des));
    phi_des   = -(1/g)*(x_ddot_cmd*sin(psi_des) - y_ddot_cmd*cos(psi_des));

    attitudeDes = [
        phi_des;
        theta_des;
        psi_des
    ];

    %% ---------------------- Informacoes auxiliares ----------------------
    controlInfo = struct();

    controlInfo.position = position;
    controlInfo.velocity = velocity;

    controlInfo.posError = posError;
    controlInfo.velError = velError;
    controlInfo.errorInt = errorIntOut;

    controlInfo.u_pos = u_pos;

    controlInfo.T = T;
    controlInfo.phi_des = phi_des;
    controlInfo.theta_des = theta_des;
    controlInfo.psi_des = psi_des;

    controlInfo.updateFreq = updateFreq;
    controlInfo.TsController = TsController;
end