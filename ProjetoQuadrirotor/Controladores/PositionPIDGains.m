function gains = PositionPIDGains()
% PositionPIDGains
% -------------------------------------------------------------------------
% Define os ganhos dos controladores P, PD e PID para posicao.
%
% Ordem dos estados de posicao:
%   position = [
%       x;
%       y;
%       z
%   ];
%
% Os valores abaixo sao iniciais e devem ser ajustados durante os testes.
%
% -------------------------------------------------------------------------

    %% ---------------------- Ganhos P ------------------------------------
    gains.P.Kp = diag([
        0.05;
        0.05;
        0.40
    ]);

    %% ---------------------- Ganhos PD -----------------------------------
    gains.PD.Kp = diag([
        0.70;   % x
        0.70;   % y
        0.90    % z
    ]);

    gains.PD.Kd = diag([
        0.90;   % x_dot
        0.90;   % y_dot
        1.40    % z_dot
    ]);

    %% ---------------------- Ganhos PID ----------------------------------
    gains.PID.Kp = diag([
        1.20;   % x
        1.20;   % y
        2.00    % z
    ]);
    
    gains.PID.Ki = diag([
        0.05;   % x
        0.05;   % y
        0.08    % z
    ]);

    gains.PID.Kd = diag([
        0.70;   % x_dot
        0.70;   % y_dot
        0.90    % z_dot
    ]);

end