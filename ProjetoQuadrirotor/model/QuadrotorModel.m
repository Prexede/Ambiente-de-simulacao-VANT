function quadConfig = QuadrotorModel(varargin)
% QuadrotorModel
% -------------------------------------------------------------------------
% Calcula os parametros nominais do quadrotor e armazena tambem limites
% fisicos usados pelo controlador e pelo mixer.
%
% Entradas principais:
%   Material     -> atualmente "CarbonFiber"
%   Geometry     -> [L W H] em cm
%   HoverSpeed   -> velocidade angular de hover [rad/s]
%   OmegaMin     -> limite inferior dos motores [rad/s]
%   OmegaMax     -> limite superior dos motores [rad/s]
%   MaxTiltAngle -> limite de inclinacao [rad]
% -------------------------------------------------------------------------

    p = inputParser;
    addParameter(p, "Material", "CarbonFiber");
    addParameter(p, "Geometry", [20 5 1], @(x) isnumeric(x) && numel(x) == 3);
    addParameter(p, "HoverSpeed", 1000, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, "OmegaMin", 0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, "OmegaMax", 5000, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, "MaxTiltAngle", deg2rad(30), @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, "MaxThrustFactor", 2.5, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parse(p, varargin{:});

    material = string(p.Results.Material);
    geometry = p.Results.Geometry(:).';
    hoverSpeed = p.Results.HoverSpeed;
    omegaMin = p.Results.OmegaMin;
    omegaMax = p.Results.OmegaMax;
    maxTiltAngle = p.Results.MaxTiltAngle;
    maxThrustFactor = p.Results.MaxThrustFactor;

    if omegaMax <= omegaMin
        error('OmegaMax deve ser maior que OmegaMin.');
    end

    L = geometry(1);
    W = geometry(2);
    H = geometry(3);

    switch lower(material)
        case "carbonfiber"
            density = 1.6; % [g/cm^3]
        otherwise
            error('Material "%s" nao implementado. Use "CarbonFiber".', material);
    end

    armLength = (L + W)/(2*100); % [m]

    volumeArmShort = L*W*H;                   % [cm^3]
    massShortArm = density*volumeArmShort/1000; % [kg]

    volumeArmLong = (2*L + W)*W*H;              % [cm^3]
    massLongArm = density*volumeArmLong/1000;   % [kg]

    frameMass = massLongArm + 2*massShortArm;
    motorMass = 0.1*frameMass;
    mass = frameMass + 4*motorMass;

    Ixx = (massLongArm/12)*((W/100)^2 + (H/100)^2);
    Iyy = (massLongArm/12)*(((2*L + W)/100)^2 + (H/100)^2);
    Izz = (massLongArm/12)*(((2*L + W)/100)^2 + (W/100)^2);

    Ixx = Ixx + 2*((massShortArm/12)*((L/100)^2 + (H/100)^2) + massShortArm*armLength^2);
    Iyy = Iyy + 2*((massShortArm/12)*((W/100)^2 + (H/100)^2));
    Izz = Izz + 2*((massShortArm/12)*((L/100)^2 + (W/100)^2) + massShortArm*armLength^2);

    IxxMotor = 0.5*motorMass*0.8*(W/200)^2 + 0.25*0.2*motorMass*(3*W/100)^2;
    IyyMotor = IxxMotor;
    IzzMotor = motorMass*0.8*(W/200)^2 + 0.5*0.2*motorMass*(3*W/100)^2;

    Ixx = Ixx + 4*IxxMotor + 2*motorMass*armLength^2;
    Iyy = Iyy + 4*IyyMotor + 2*motorMass*armLength^2;
    Izz = Izz + 4*(IzzMotor + motorMass*armLength^2);

    inertia = diag([Ixx, Iyy, Izz]);

    gravity = 9.81;
    kThrust = mass*gravity/(4*hoverSpeed^2);
    kDrag = kThrust/10;

    quadConfig = struct();

    quadConfig.material = material;
    quadConfig.density = density;

    quadConfig.geometry.sizeCm = geometry;
    quadConfig.geometry.L = L;
    quadConfig.geometry.W = W;
    quadConfig.geometry.H = H;

    quadConfig.mass = mass;
    quadConfig.nominalMass = mass;
    quadConfig.frameMass = frameMass;
    quadConfig.motorMass = motorMass;
    quadConfig.armLength = armLength;
    quadConfig.inertia = inertia;
    quadConfig.gravity = gravity;
    quadConfig.kThrust = kThrust;
    quadConfig.kDrag = kDrag;
    quadConfig.hoverSpeed = hoverSpeed;

    quadConfig.actuator.omegaMin = omegaMin;
    quadConfig.actuator.omegaMax = omegaMax;

    quadConfig.control.maxTiltAngle = maxTiltAngle;
    quadConfig.control.minThrust = 0;
    quadConfig.control.maxThrust = maxThrustFactor*mass*gravity;
    quadConfig.control.maxThrustFactor = maxThrustFactor;

    % Aliases mantidos para facilitar compatibilidade com funcoes antigas.
    quadConfig.Inertia = inertia;
    quadConfig.grav = gravity;
    quadConfig.k_aero = kThrust;
    quadConfig.k_drag = kDrag;
    quadConfig.Prop = IzzMotor;
end
