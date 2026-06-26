function gains = PositionGains()

    gains.P.Kp = diag([0.05, 0.05, 0.05]);

    gains.PD.Kp = diag([0.09, 0.09, 0.60]);
    gains.PD.Kd = diag([0.75, 0.75, 1.45]);

    gains.PID.Kp = diag([0.075, 0.075, 1.15]);
    gains.PID.Ki = diag([0.0003, 0.0003, 0.45]);
    %gains.PID.Ki = diag([0.0003, 0.0003, 0.006]);
    gains.PID.Kd = diag([0.80, 0.80, 2.40]);

end