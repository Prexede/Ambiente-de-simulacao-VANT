function u = PIDController(error, errorDot, errorIntegral, gains)
% PIDController
% -------------------------------------------------------------------------
% u = Kp*erro + Kd*erro_derivativo + Ki*integral_erro
% -------------------------------------------------------------------------
    u = gains.Kp*error + gains.Kd*errorDot + gains.Ki*errorIntegral;
end
