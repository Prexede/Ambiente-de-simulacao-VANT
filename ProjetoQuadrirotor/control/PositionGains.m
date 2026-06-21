function gains = PositionGains()
% PositionGains
% -------------------------------------------------------------------------
% Ganhos iniciais dos controladores de posicao.
% Ordem: [x y z].
% -------------------------------------------------------------------------

    gains.P.Kp = diag([0.05, 0.05, 0.40]);

    gains.PD.Kp = diag([0.70, 0.70, 0.90]);
    gains.PD.Kd = diag([0.90, 0.90, 1.40]);

    gains.PID.Kp = diag([1.20, 1.20, 2.00]);
    gains.PID.Ki = diag([0.05, 0.05, 0.08]);
    gains.PID.Kd = diag([0.70, 0.70, 0.90]);
end
