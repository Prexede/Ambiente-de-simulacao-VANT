function mracParams = MRACParams(quadConfig)
% MRACParams
% -------------------------------------------------------------------------
% Parametros do MRAC usado no modelo MRAC_Test.
%
% O controle adaptativo foi organizado em quatro canais de segunda ordem:
%   z, phi, theta e psi.
%
% Cada canal possui:
%   xRef      -> estado do modelo de referencia [y_ref; y_ref_dot]
%   Aref/Bref -> modelo de referencia de segunda ordem
%   P/B       -> matrizes usadas no termo e'*P*B da lei adaptativa
%   Kx0/Kr0   -> ganhos iniciais nominais
%   gamma     -> taxas de adaptacao
% -------------------------------------------------------------------------

    zetaAtt = 0.95;
    wnAtt = 5.0;
    
    zetaZ = 0.80;
    wnZ = 8.0;

    Ix = quadConfig.inertia(1,1);
    Iy = quadConfig.inertia(2,2);
    Iz = quadConfig.inertia(3,3);
    m = quadConfig.mass;

    mracParams = struct();

    mracParams.altitude = BuildChannelParams( ...
        zetaZ, ...
        wnZ, ...
        m*[-wnZ^2; -2*zetaZ*wnZ], ...
        m*wnZ^2, ...
        diag([0.020, 0.020]), ...
        0.020, ...
        0.005, ...
        0.000);

    mracParams.attitude.roll = BuildChannelParams( ...
        zetaAtt, ...
        wnAtt, ...
        Ix*[-wnAtt^2; -2*zetaAtt*wnAtt], ...
        Ix*wnAtt^2, ...
        diag([0.010, 0.010]), ...
        0.010, ...
        0.003, ...
        0.0005);

    mracParams.attitude.pitch = BuildChannelParams( ...
        zetaAtt, ...
        wnAtt, ...
        Iy*[-wnAtt^2; -2*zetaAtt*wnAtt], ...
        Iy*wnAtt^2, ...
        diag([0.010, 0.010]), ...
        0.010, ...
        0.003, ...
        0.0005);

    mracParams.attitude.yaw = BuildChannelParams( ...
        zetaAtt, ...
        wnAtt, ...
        Iz*[-wnAtt^2; -2*zetaAtt*wnAtt], ...
        Iz*wnAtt^2, ...
        diag([0.010, 0.010]), ...
        0.010, ...
        0.003, ...
        0.0005);
end

function params = BuildChannelParams(zeta, wn, Kx0, Kr0, gammaX, gammaR, gammaD, gammaO)
    Aref = [0, 1; -wn^2, -2*zeta*wn];
    Bref = [0; wn^2];
    B = [0; 1];
    Q = eye(2);
    P = SolveLyapunov2x2(Aref, Q);

    params = struct();
    params.zeta = zeta;
    params.wn = wn;
    params.Aref = Aref;
    params.Bref = Bref;
    params.B = B;
    params.Q = Q;
    params.P = P;

    params.Kx0 = Kx0(:);
    params.Kr0 = Kr0;
    params.Kd0 = 0;
    params.OHat0 = 0;

    params.gammaX = gammaX;
    params.gammaR = gammaR;
    params.gammaD = gammaD;
    params.gammaO = gammaO;

    params.enableProjection = true;
    params.KxLimit = max(10*abs(params.Kx0), 0.5*ones(2,1));
    params.KrLimit = max(10*abs(params.Kr0), 0.5);
    params.KdLimit = max(10*abs(params.Kr0), 0.5);
    params.OLimit = max(10*abs(params.Kr0), 0.5);
end

function P = SolveLyapunov2x2(A, Q)
    L = kron(eye(2), A') + kron(A', eye(2));
    P = reshape(-L\Q(:), 2, 2);
    P = 0.5*(P + P');
end
