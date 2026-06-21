function u = PDController(error, errorDot, gains)
% PDController
% -------------------------------------------------------------------------
% Controlador proporcional-derivativo generico.
%
% ENTRADAS:
%   error    : vetor de erro
%   errorDot : derivada do erro
%   gains    : struct com os campos:
%              gains.Kp
%              gains.Kd
%
% SAIDA:
%   u : sinal de controle
%
% Lei de controle:
%   u = Kp*error + Kd*errorDot
%
% EXEMPLO:
%   gains = AttitudePIDGains();
%   u = PDController(error, errorDot, gains.PD);
%
% -------------------------------------------------------------------------

    u = gains.Kp*error + gains.Kd*errorDot;
end