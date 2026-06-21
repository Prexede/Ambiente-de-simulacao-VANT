function u = PIDController(error, errorDot, errorInt, gains)
% PIDController
% -------------------------------------------------------------------------
% Controlador proporcional-integral-derivativo generico.
%
% ENTRADAS:
%   error    : vetor de erro
%   errorDot : derivada do erro
%   errorInt : integral do erro
%   gains    : struct com os campos:
%              gains.Kp
%              gains.Ki
%              gains.Kd
%
% SAIDA:
%   u : sinal de controle
%
% Lei de controle:
%   u = Kp*error + Ki*errorInt + Kd*errorDot
%
% EXEMPLO:
%   gains = PositionPIDGains();
%   u = PIDController(error, errorDot, errorInt, gains.PID);
%
% -------------------------------------------------------------------------

    u = gains.Kp*error + gains.Ki*errorInt + gains.Kd*errorDot;
end