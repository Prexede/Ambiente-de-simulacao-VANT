function gains = AttitudeGains()
% AttitudeGains
% -------------------------------------------------------------------------
% Ganhos iniciais dos controladores de atitude.
% Ordem: [phi theta psi].
% -------------------------------------------------------------------------

    gains.P.Kp = diag([4.0, 4.0, 2.0]);

    gains.PD.Kp = diag([0.05, 0.05, 0.02]);
    gains.PD.Kd = diag([0.01, 0.01, 0.005]);

    gains.PID.Kp = diag([0.60, 0.60, 1.00]);
    gains.PID.Ki = diag([0.005, 0.005, 0.015]);
    gains.PID.Kd = diag([0.90, 0.90, 1.30]);
end
