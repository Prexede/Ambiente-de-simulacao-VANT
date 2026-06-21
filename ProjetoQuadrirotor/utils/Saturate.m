function y = Saturate(u, lowerLimit, upperLimit)
% Saturate
% -------------------------------------------------------------------------
% Limita u entre lowerLimit e upperLimit. Funciona com escalares, vetores
% e matrizes, desde que as dimensoes sejam compativeis.
% -------------------------------------------------------------------------
    y = min(max(u, lowerLimit), upperLimit);
end
