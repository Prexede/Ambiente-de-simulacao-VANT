function gains = AttitudeGains()

    gains.P.Kp = diag([0.30, 0.30, 0.18]);

    gains.PD.Kp = diag([0.28, 0.28, 0.18]);
    gains.PD.Kd = diag([0.12, 0.12, 0.16]);

    gains.PID.Kp = diag([0.22, 0.22, 0.14]);
    gains.PID.Ki = diag([0.0010, 0.0010, 0.0008]);
    gains.PID.Kd = diag([0.15, 0.15, 0.18]);

end