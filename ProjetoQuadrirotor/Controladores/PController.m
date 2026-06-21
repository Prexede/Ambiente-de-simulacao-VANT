function u = PController(error, gains)
% PController
% -------------------------------------------------------------------------
% Controlador proporcional generico.
%
% ENTRADAS:
%   error : vetor de erro
%   gains : struct com o campo:
%           gains.Kp
%
% SAIDA:
%   u : sinal de controle
%
% Lei de controle:
%   u = Kp*error
%
% EXEMPLO:
%   gains = AttitudePIDGains();
%   u = PController(error, gains.P);
%
% -------------------------------------------------------------------------

    u = gains.Kp*error;
end