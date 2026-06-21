function [I_til, quad] = EstimatedQuadParameters(material, droneGeom, w_hover)
% EstimatedQuadParameters
% -------------------------------------------------------------------------
% Estima os parametros principais de um quadrirotor simples:
%   - tensor de inercia;
%   - massa total aproximada;
%   - comprimento efetivo do braco;
%   - coeficientes aerodinamicos estimados.
%
% ENTRADAS:
%   material  : material utilizado. Atualmente: "CarbonFiber"
%   droneGeom : vetor [L W H], em centimetros
%               L -> comprimento do braco [cm]
%               W -> largura do braco [cm]
%               H -> altura/espessura do braco [cm]
%   w_hover   : velocidade angular nominal de hover [rad/s]
%
% SAIDAS:
%   I_til : tensor de inercia 3x3 [kg.m^2]
%   quad  : struct com parametros estimados
%
% EXEMPLO:
%   [I_til, quad] = EstimatedQuadParameters("CarbonFiber", [20 5 1], 1000);
%
% -------------------------------------------------------------------------

    %% ---------------------- Linhas opcionais para teste -----------------
    % Mude o valor da variavel teste caso necessário
    teste = false;

    if teste == 1
        material = "CarbonFiber";
        droneGeom = [20 5 1]; % [L W H] em centimetros
        w_hover = 1000;       % [rad/s]
    end

    %% ---------------------- Tratamento minimo de entrada ----------------
    if ~exist('material') || isempty(material)
        error('Informe o material. Use "CarbonFiber".');
    end

    if ~exist('droneGeom') || isempty(droneGeom)
        error('Informe droneGeom como vetor [L W H], em centimetros.');
    end

    if ~exist('w_hover') || isempty(w_hover)
        error('Informe w_hover em rad/s.');
    end

    %% ---------------------- Geometria em centimetros --------------------
    L = droneGeom(1);  
    W = droneGeom(2);    %[cm]
    H = droneGeom(3);   

    % Comprimento efetivo do braco em metros.
    % Representa a distancia aproximada entre o centro do drone
    % e o centro do conjunto motor/helice.
    armLength = (L + W)/(2*100); % [m]

    %% ---------------------- Material ------------------------------------
    if material == "CarbonFiber"
        rho = 1.6; % [g/cm^3]
    else
        error('Material nao implementado. Use "CarbonFiber".');
    end

    %% ---------------------- Massa da estrutura --------------------------
    % Braco longo na direcao x: dimensoes (2L + W) x W x H.
    % Dois bracos curtos na direcao y.
    volume_arm_short = L*W*H;                 % [cm^3]
    mass_s_arm = rho*volume_arm_short/1000;   % [kg]

    volume_arm_long = (2*L + W)*W*H;          % [cm^3]
    mass_l_arm = rho*volume_arm_long/1000;    % [kg]

    frame_mass = mass_l_arm + 2*mass_s_arm;   % [kg]

    % Hipotese cada conjunto motor + helice possui 10% da massa do frame.
    motor_mass = 0.1*frame_mass;              % [kg]

    overall_mass = frame_mass + 4*motor_mass; % [kg]

    %% ---------------------- Tensor de inercia do frame ------------------
    % Braço longo
    Ixx = (mass_l_arm/12)*((W/100)^2 + (H/100)^2);
    Iyy = (mass_l_arm/12)*(((2*L + W)/100)^2 + (H/100)^2);
    Izz = (mass_l_arm/12)*(((2*L + W)/100)^2 + (W/100)^2);

    %Braço curto
    % O termo armLength^2 vem do teorema dos eixos paralelos.
    Ixx = Ixx + 2*((mass_s_arm/12)*((L/100)^2 + (H/100)^2) + ...
                   mass_s_arm*armLength^2);

    Iyy = Iyy + 2*((mass_s_arm/12)*((W/100)^2 + (H/100)^2));

    Izz = Izz + 2*((mass_s_arm/12)*((L/100)^2 + (W/100)^2) + ...
                   mass_s_arm*armLength^2);

    %% ---------------------- Tensor dos motores/helices ------------------
    % Hipoteses usadas no codigo de referencia:
    % - 80% da massa do conjunto e atribuida ao motor;
    % - 20% da massa do conjunto e atribuida a helice;
    % - diametro do motor = W;
    % - diametro da helice = 6W.

    Ixx_motor = 0.5*motor_mass*0.8*(W/200)^2 + ...
                0.25*0.2*motor_mass*(3*W/100)^2;

    Iyy_motor = Ixx_motor;

    Izz_motor = motor_mass*0.8*(W/200)^2 + ...
                0.5*0.2*motor_mass*(3*W/100)^2;

    %% ---------------------- Tensor de inercia total ---------------------
    Ixx = Ixx + 4*Ixx_motor + 2*motor_mass*armLength^2;
    Iyy = Iyy + 4*Iyy_motor + 2*motor_mass*armLength^2;
    Izz = Izz + 4*(Izz_motor + motor_mass*armLength^2);

    I_til = [
        Ixx, 0,   0;
        0,   Iyy, 0;
        0,   0,   Izz
    ];

    %% ---------------------- Coeficientes aerodinamicos ------------------
    % Estimativa pela condicao de hover:
    %
    %   4*k_aero*w_hover^2 = m*g (4 motores e vel angular)
    %
    % Neste modelo, w_hover entra como parametro informado pelo usuario.
    % A partir dele, estima-se k_aero:
    %
    %   k_aero = m*g/(4*w_hover^2)

    g = 9.81; % [m/s^2]

    % Empuxo 
    k_aero = overall_mass*g/(4*w_hover^2);

    % Arrasto
    k_drag = k_aero/10;

    %% ---------------------- Struct de saida -----------------------------
    quad = struct();

    quad.material = material;
    quad.density = rho;

    quad.Inertia = I_til;
    quad.mass = overall_mass;
    quad.armLength = armLength;
    quad.Prop = Izz_motor;
    quad.grav = g;
    quad.k_aero = k_aero;
    quad.k_drag = k_drag;
end