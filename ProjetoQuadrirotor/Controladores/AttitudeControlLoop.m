function [tau, errorIntOut, controlInfo] = AttitudeControlLoop(state, attitudeDes, controlType, gains, updateFreq, errorIntIn)
% AttitudeControlLoop
% -------------------------------------------------------------------------
% Controlador de atitude do quadrirotor.
%
% Calcula:
%   tau = [tau_x; tau_y; tau_z]
%
% ENTRADAS:
%   state       : vetor de estados 12x1
%   attitudeDes : atitude desejada 3x1
%   controlType : "P", "PD" ou "PID"
%   gains       : struct gerada por AttitudePIDGains()
%   updateFreq  : frequencia de atualizacao do controlador [Hz]
%
% ENTRADA OPCIONAL:
%   errorIntIn  : matriz 3x1 com a integral do erro de atitude
%
% SAIDAS:
%   tau         : torques desejados 3x1
%   errorIntOut : matriz 3x1 com a integral atualizada
%   controlInfo : struct auxiliar
%
% -------------------------------------------------------------------------

    %% ---------------------- Tratamento minimo de entrada ----------------
    if nargin < 1 || isempty(state)
        error('Informe o vetor de estados state.');
    end

    if nargin < 2 || isempty(attitudeDes)
        error('Informe attitudeDes.');
    end

    if nargin < 3 || isempty(controlType)
        error('Informe controlType: "P", "PD" ou "PID".');
    end

    if nargin < 4 || isempty(gains)
        error('Informe os ganhos gerados por AttitudePIDGains().');
    end

    if nargin < 5 || isempty(updateFreq)
        error('Informe updateFreq em Hz.');
    end

    %% ---------------------- Entrada opcional ----------------------------
    if nargin < 6 || isempty(errorIntIn)
        errorIntIn = zeros(3,1);
    end

    %% ---------------------- Periodo equivalente -------------------------
    TsController = 1/updateFreq;

    %% ---------------------- Estados de atitude --------------------------
    attitude = [
        state(7);
        state(8);
        state(9)
    ];

    angularRate = [
        state(10);
        state(11);
        state(12)
    ];

    %% ---------------------- Erros de atitude ----------------------------
    attError = attitudeDes - attitude;

    % Corrige erro de yaw para ficar entre -pi e pi.
    attError(3) = atan2(sin(attError(3)), cos(attError(3)));

    % Por enquanto, as taxas desejadas sao zero.
    angularRateDes = zeros(3,1);
    rateError = angularRateDes - angularRate;

    %% ---------------------- Controlador generico ------------------------
    controlType = string(controlType);

    switch controlType
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
            error('controlType nao implementado. Use "P", "PD" ou "PID".');
    end

    %% ---------------------- Informacoes auxiliares ----------------------
    controlInfo = struct();

    controlInfo.attitude = attitude;
    controlInfo.angularRate = angularRate;

    controlInfo.attitudeDes = attitudeDes;
    controlInfo.angularRateDes = angularRateDes;

    controlInfo.attError = attError;
    controlInfo.rateError = rateError;
    controlInfo.errorInt = errorIntOut;

    controlInfo.tau = tau;

    controlInfo.updateFreq = updateFreq;
    controlInfo.TsController = TsController;
end