function angleWrapped = WrapAngle(angle)
% WrapAngle
% -------------------------------------------------------------------------
% Normaliza angulos para o intervalo [-pi, pi].
% -------------------------------------------------------------------------
    angleWrapped = atan2(sin(angle), cos(angle));
end
