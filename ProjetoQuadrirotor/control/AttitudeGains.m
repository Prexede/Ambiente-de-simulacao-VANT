function gains = AttitudeGains()
% AttitudeGains
% -------------------------------------------------------------------------
% Ganhos iniciais dos controladores de atitude.
% Ordem: [phi theta psi].
% -------------------------------------------------------------------------

    gains.P.Kp = diag([4.0, 4.0, 2.0]);

    gains.PD.Kp = diag([0.05, 0.05, 0.02]);
    gains.PD.Kd = diag([0.01, 0.01, 0.005]);

    gains.PID.Kp = diag([8.0, 8.0, 4.0]);
    gains.PID.Ki = diag([0.10, 0.10, 0.05]);
    gains.PID.Kd = diag([2.0, 2.0, 1.0]);
end
