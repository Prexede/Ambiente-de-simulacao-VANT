function mracParams = MRACParams(quadConfig)
% MRACParams
% -------------------------------------------------------------------------
% Parametros do controlador MRAC.
%
% Ajustes principais:
%   - zeta e wn da altitude
%   - zeta e wn da atitude
%   - um conjunto de gammas para altitude
%   - um conjunto de gammas para atitude
% -------------------------------------------------------------------------

    %% Modelo de referencia

    zetaAtt = 0.95;
    wnAtt   = 5.0;

    zetaZ = 0.80;
    wnZ   = 8.0;


    %% Fatores de adaptacao da altitude

    gammaAlt.x = 0.05;   % KxHat: estados [z; zDot]
    gammaAlt.r = 0.05;   % KrHat: referencia z_cmd
    gammaAlt.d = 10.00;   % KdHat: disturbio constante, importante para massa
    gammaAlt.o = 0.00;   % OHat: nao usado na altitude


    %% Fatores de adaptacao da atitude

    gammaAtt.x = 0.02;   % KxHat: estados [angulo; velocidade angular]
    gammaAtt.r = 0.02;   % KrHat: referencia angular
    gammaAtt.d = 0.01;   % KdHat: disturbio constante
    gammaAtt.o = 0.00;   % OHat: acoplamento. deixe 0 por enquanto


    %% Limites da projecao adaptativa

    enableProjection = true;

    limitsAlt.Kx = 200.0;
    limitsAlt.Kr = 200.0;
    limitsAlt.Kd = 100.0;
    limitsAlt.O  = 0.0;

    limitsAtt.Kx = 20.0;
    limitsAtt.Kr = 20.0;
    limitsAtt.Kd = 5.0;
    limitsAtt.O  = 0.0;


    %% Parametros fisicos

    m  = quadConfig.mass;
    Ix = quadConfig.inertia(1,1);
    Iy = quadConfig.inertia(2,2);
    Iz = quadConfig.inertia(3,3);


    %% Ganhos iniciais nominais

    Kx0_z = m*[-wnZ^2; -2*zetaZ*wnZ];
    Kr0_z = m*wnZ^2;

    Kx0_phi = Ix*[-wnAtt^2; -2*zetaAtt*wnAtt];
    Kr0_phi = Ix*wnAtt^2;

    Kx0_theta = Iy*[-wnAtt^2; -2*zetaAtt*wnAtt];
    Kr0_theta = Iy*wnAtt^2;

    Kx0_psi = Iz*[-wnAtt^2; -2*zetaAtt*wnAtt];
    Kr0_psi = Iz*wnAtt^2;


    %% Montagem dos canais

    mracParams = struct();

    mracParams.altitude = BuildChannelParams( ...
        zetaZ, ...
        wnZ, ...
        Kx0_z, ...
        Kr0_z, ...
        gammaAlt, ...
        limitsAlt, ...
        enableProjection);

    mracParams.attitude = struct();

    mracParams.attitude.roll = BuildChannelParams( ...
        zetaAtt, ...
        wnAtt, ...
        Kx0_phi, ...
        Kr0_phi, ...
        gammaAtt, ...
        limitsAtt, ...
        enableProjection);

    mracParams.attitude.pitch = BuildChannelParams( ...
        zetaAtt, ...
        wnAtt, ...
        Kx0_theta, ...
        Kr0_theta, ...
        gammaAtt, ...
        limitsAtt, ...
        enableProjection);

    mracParams.attitude.yaw = BuildChannelParams( ...
        zetaAtt, ...
        wnAtt, ...
        Kx0_psi, ...
        Kr0_psi, ...
        gammaAtt, ...
        limitsAtt, ...
        enableProjection);
end


function params = BuildChannelParams(zeta, wn, Kx0, Kr0, gamma, limits, enableProjection)

    Aref = [ ...
        0,       1; ...
       -wn^2,  -2*zeta*wn];

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
    params.Kd0 = 0.0;
    params.OHat0 = 0.0;

    params.gammaX = gamma.x;
    params.gammaR = gamma.r;
    params.gammaD = gamma.d;
    params.gammaO = gamma.o;

    params.enableProjection = enableProjection;

    params.KxLimit = limits.Kx*ones(2,1);
    params.KrLimit = limits.Kr;
    params.KdLimit = limits.Kd;
    params.OLimit  = limits.O;
end


function P = SolveLyapunov2x2(A, Q)
% Resolve:
%   P*A + A'*P = -Q

    L = kron(eye(2), A') + kron(A', eye(2));
    P = reshape(-L\Q(:), 2, 2);
    P = 0.5*(P + P');
end