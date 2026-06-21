function u = PDController(error, errorDot, gains)
% PDController
% -------------------------------------------------------------------------
% u = Kp*erro + Kd*erro_derivativo
% -------------------------------------------------------------------------
    u = gains.Kp*error + gains.Kd*errorDot;
end
