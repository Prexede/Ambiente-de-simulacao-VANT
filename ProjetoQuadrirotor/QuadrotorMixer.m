function [motorOmega, omegaSquared] = QuadrotorMixer(T, tau, quad)
% QuadrotorMixer
% -------------------------------------------------------------------------
% Converte o empuxo total e os torques desejados em velocidades angulares
% dos quatro motores do quadrirotor.
%
% ENTRADAS:
%   T    : empuxo total desejado [N]
%
%   tau  : vetor de torques desejados [N.m]
%          tau = [
%              tau_x;
%              tau_y;
%              tau_z
%          ];
%
%   quad : struct gerada por EstimatedQuadParameters
%
% SAIDAS:
%   motorOmega   : velocidades angulares dos motores [rad/s]
%                  motorOmega = [
%                      omega_1;
%                      omega_2;
%                      omega_3;
%                      omega_4
%                  ];
%
%   omegaSquared : quadrado das velocidades angulares dos motores
%
% CONVENCAO:
%   T     = k_aero*(omega_1^2 + omega_2^2 + omega_3^2 + omega_4^2)
%   tau_x = l*k_aero*(-omega_2^2 + omega_4^2)
%   tau_y = l*k_aero*( omega_1^2 - omega_3^2)
%   tau_z = k_drag*(-omega_1^2 + omega_2^2 - omega_3^2 + omega_4^2)
%
% -------------------------------------------------------------------------

    %% ---------------------- Parametros do quadrirotor -------------------
    k_aero = quad.k_aero;
    k_drag = quad.k_drag;
    l = quad.armLength;

    %% ---------------------- Limites hardcoded dos motores ---------------
    omegaMin = 0;                    % [rad/s]
    omegaMax = 5*1000;  % [rad/s]

    omegaSquaredMin = omegaMin^2;
    omegaSquaredMax = omegaMax^2;

    %% ---------------------- Vetor de comandos ---------------------------
    U = [
        T;
        tau(1);
        tau(2);
        tau(3)
    ];
    

    %% ---------------------- Matriz de mistura ---------------------------
    mixerMatrix = [
        k_aero,        k_aero,        k_aero,        k_aero;
        0,            -l*k_aero,      0,             l*k_aero;
        l*k_aero,      0,            -l*k_aero,      0;
       -k_drag,        k_drag,       -k_drag,        k_drag
    ];

    %% ---------------------- Calculo de omega^2 --------------------------
    omegaSquared = mixerMatrix\U;

    %% ---------------------- Saturacao fisica ----------------------------
    % Evita omega_i^2 negativo.
    omegaSquared = max(omegaSquared, omegaSquaredMin);

    % Evita velocidade angular acima do limite maximo.
    omegaSquared = min(omegaSquared, omegaSquaredMax);

    %% ---------------------- Velocidade angular dos motores --------------
    motorOmega = sqrt(omegaSquared);

    %% ---------------------- Comando real apos saturacao -----------------
    U_real = mixerMatrix*omegaSquared;
end