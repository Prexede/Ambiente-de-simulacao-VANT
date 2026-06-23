function [u, mracState] = MRACSecondOrder(y, yDot, command, phiBasis, mracParams, mracState, dt)
% MRACSecondOrder
% -------------------------------------------------------------------------
% Nucleo generico do MRAC para um subsistema de segunda ordem.
%
% Planta tratada como:
%   x = [y; yDot]
%
% Modelo de referencia:
%   xRefDot = Aref*xRef + Bref*command
%
% Lei de controle:
%   u = KxHat'*x + KrHat*command - OHat*phiBasis - KdHat
% -------------------------------------------------------------------------

    if nargin < 4 || isempty(phiBasis)
        phiBasis = 0;
    end

    if nargin < 6 || isempty(mracState)
        mracState = InitializeMRACSecondOrderState(y, yDot, mracParams);
    end

    xPlant = [y; yDot];

    xRefDot = mracParams.Aref*mracState.xRef + mracParams.Bref*command;
    mracState.xRef = mracState.xRef + dt*xRefDot;

    errorState = xPlant - mracState.xRef;
    sigma = errorState.'*mracParams.P*mracParams.B;

    KxDot = -mracParams.gammaX*xPlant*sigma;
    KrDot = -mracParams.gammaR*command*sigma;
    KdDot =  mracParams.gammaD*sigma;
    ODot  =  mracParams.gammaO*phiBasis*sigma;

    mracState.KxHat = mracState.KxHat + dt*KxDot;
    mracState.KrHat = mracState.KrHat + dt*KrDot;
    mracState.KdHat = mracState.KdHat + dt*KdDot;
    mracState.OHat  = mracState.OHat  + dt*ODot;

    if isfield(mracParams, "enableProjection") && mracParams.enableProjection
        mracState.KxHat = Saturate(mracState.KxHat, -mracParams.KxLimit, mracParams.KxLimit);
        mracState.KrHat = Saturate(mracState.KrHat, -mracParams.KrLimit, mracParams.KrLimit);
        mracState.KdHat = Saturate(mracState.KdHat, -mracParams.KdLimit, mracParams.KdLimit);
        mracState.OHat  = Saturate(mracState.OHat,  -mracParams.OLimit,  mracParams.OLimit);
    end

    u = mracState.KxHat.'*xPlant + ...
        mracState.KrHat*command - ...
        mracState.OHat*phiBasis - ...
        mracState.KdHat;

    mracState.error = errorState;
    mracState.sigma = sigma;
    mracState.command = command;
    mracState.phiBasis = phiBasis;
    mracState.u = u;
end

function mracState = InitializeMRACSecondOrderState(y, yDot, mracParams)
    mracState = struct();
    mracState.xRef = [y; yDot];
    mracState.KxHat = mracParams.Kx0(:);
    mracState.KrHat = mracParams.Kr0;
    mracState.KdHat = mracParams.Kd0;
    mracState.OHat = mracParams.OHat0;
    mracState.error = zeros(2,1);
    mracState.sigma = 0;
    mracState.command = y;
    mracState.phiBasis = 0;
    mracState.u = 0;
end
