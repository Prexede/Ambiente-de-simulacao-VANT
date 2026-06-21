function gains = PositionGains()
% PositionGains
% -------------------------------------------------------------------------
% Ganhos iniciais dos controladores de posicao.
% Ordem: [x y z].
% -------------------------------------------------------------------------

    gains.P.Kp = diag([0.05, 0.05, 0.05]);

    gains.PD.Kp = diag([0.55, 0.55, 0.90]);
    gains.PD.Kd = diag([0.90, 0.90, 1.30]);

    gains.PID.Kp = diag([0.60, 0.60, 1.00]);
    gains.PID.Ki = diag([0.005, 0.005, 0.015]);
    gains.PID.Kd = diag([0.90, 0.90, 1.30]);
end
