function stateDot = QuadrotorDynamics(state, motorOmega, quadConfig, disturbance)
% QuadrotorDynamics
% -------------------------------------------------------------------------
% Dinamica nao linear simplificada do quadrotor.
%
% Estado:
%   [x y z x_dot y_dot z_dot phi theta psi p q r]'
%
% Convencoes:
%   z positivo para cima;
%   empuxo atua no eixo z do corpo;
%   rotacao corpo->inercial: R = Rz(psi)*Ry(theta)*Rx(phi).
% -------------------------------------------------------------------------

    if nargin < 4 || isempty(disturbance)
        disturbance = DefaultDisturbance();
    else
        disturbance = CompleteDisturbance(disturbance);
    end

    state = state(:);
    motorOmega = motorOmega(:);

    if numel(state) ~= 12
        error('state deve ser um vetor 12x1.');
    end

    if numel(motorOmega) ~= 4
        error('motorOmega deve ser um vetor 4x1.');
    end

    idx = StateIndex();

    positionDot = state(idx.velocity);

    phi = state(idx.phi);
    theta = state(idx.theta);
    psi = state(idx.psi);

    omegaBody = state(idx.bodyRate);

    mass = GetQuadField(quadConfig, "mass");
    inertia = GetQuadField(quadConfig, "inertia", "Inertia");
    armLength = GetQuadField(quadConfig, "armLength");
    gravity = GetQuadField(quadConfig, "gravity", "grav");
    kThrust = GetQuadField(quadConfig, "kThrust", "k_aero");
    kDrag = GetQuadField(quadConfig, "kDrag", "k_drag");

    if mass <= 0
        error('quadConfig.mass deve ser positivo.');
    end

    omegaSquared = motorOmega.^2;

    T1 = kThrust*omegaSquared(1);
    T2 = kThrust*omegaSquared(2);
    T3 = kThrust*omegaSquared(3);
    T4 = kThrust*omegaSquared(4);

    thrustTotal = T1 + T2 + T3 + T4;

    tauMotor = [
        armLength*(T4 - T2);
        armLength*(T1 - T3);
        kDrag*(-omegaSquared(1) + omegaSquared(2) - omegaSquared(3) + omegaSquared(4))
    ];

    R = RotationMatrixZYX(phi, theta, psi);

    forceThrustInertial = R*[0; 0; thrustTotal];
    forceGravityInertial = [0; 0; -mass*gravity];
    forceExternalInertial = disturbance.forceInertial;
    forceExternalBodyAsInertial = R*disturbance.forceBody;

    acceleration = (forceThrustInertial + ...
                    forceGravityInertial + ...
                    forceExternalInertial + ...
                    forceExternalBodyAsInertial)/mass;

    attitudeDot = EulerRatesZYX(phi, theta, omegaBody);

    omegaDot = inertia \ (tauMotor - cross(omegaBody, inertia*omegaBody));

    stateDot = [
        positionDot;
        acceleration;
        attitudeDot;
        omegaDot
    ];
end

function disturbance = DefaultDisturbance()
    disturbance.forceInertial = zeros(3,1);
    disturbance.forceBody = zeros(3,1);
end

function disturbance = CompleteDisturbance(disturbance)
    if ~isstruct(disturbance)
        error('disturbance deve ser uma struct.');
    end

    disturbance.forceInertial = ReadVector3Field(disturbance, ...
        ["forceInertial", "windForceInertial", "forceWorld"], zeros(3,1));

    disturbance.forceBody = ReadVector3Field(disturbance, ...
        ["forceBody", "windForceBody"], zeros(3,1));
end

function value = ReadVector3Field(s, names, defaultValue)
    value = defaultValue;

    for i = 1:numel(names)
        fieldName = char(names(i));

        if isfield(s, fieldName)
            candidate = s.(fieldName);

            if isempty(candidate)
                value = defaultValue;
            elseif isnumeric(candidate) && numel(candidate) == 3
                value = candidate(:);
            else
                error('%s deve ser um vetor 3x1.', fieldName);
            end

            return;
        end
    end
end

function value = GetQuadField(s, primaryName, aliasName)
    if isfield(s, char(primaryName))
        value = s.(char(primaryName));
        return;
    end

    if nargin >= 3 && isfield(s, char(aliasName))
        value = s.(char(aliasName));
        return;
    end

    error('Campo %s nao encontrado em quadConfig.', primaryName);
end

function R = RotationMatrixZYX(phi, theta, psi)
    cphi = cos(phi);
    sphi = sin(phi);

    ctheta = cos(theta);
    stheta = sin(theta);

    cpsi = cos(psi);
    spsi = sin(psi);

    Rz = [
        cpsi, -spsi, 0;
        spsi,  cpsi, 0;
        0,     0,    1
    ];

    Ry = [
        ctheta, 0, stheta;
        0,      1, 0;
       -stheta, 0, ctheta
    ];

    Rx = [
        1, 0,     0;
        0, cphi, -sphi;
        0, sphi,  cphi
    ];

    R = Rz*Ry*Rx;
end

function etaDot = EulerRatesZYX(phi, theta, omegaBody)
    p = omegaBody(1);
    q = omegaBody(2);
    r = omegaBody(3);

    ctheta = cos(theta);

    if abs(ctheta) < 1e-6
        ctheta = sign(ctheta + eps)*1e-6;
    end

    etaDot = [
        p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
        q*cos(phi) - r*sin(phi);
        q*sin(phi)/ctheta + r*cos(phi)/ctheta
    ];
end