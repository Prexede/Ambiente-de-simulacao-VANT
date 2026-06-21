function gains = AttitudePIDGains()
% AttitudePIDGains
% -------------------------------------------------------------------------
% Define os ganhos dos controladores P, PD e PID para atitude.
%
% Ordem dos estados de atitude:
%   attitude = [
%       phi;
%       theta;
%       psi
%   ];
%
% Ordem da saida:
%   tau = [
%       tau_x;
%       tau_y;
%       tau_z
%   ];
% -------------------------------------------------------------------------

    %% ---------------------- Ganhos P ------------------------------------
    gains.P.Kp = diag([
        4.0;    % phi
        4.0;    % theta
        2.0     % psi
    ]);

    %% ---------------------- Ganhos PD -----------------------------------
    gains.PD.Kp = diag([
        0.05;
        0.05;
        0.02
    ]);
    
    gains.PD.Kd = diag([
        0.01;
        0.01;
        0.005
    ]);

    %% ---------------------- Ganhos PID ----------------------------------
    gains.PID.Kp = diag([
        8.0;    % phi
        8.0;    % theta
        4.0     % psi
    ]);

    gains.PID.Ki = diag([
        0.10;   % phi
        0.10;   % theta
        0.05    % psi
    ]);

    gains.PID.Kd = diag([
        2.0;    % phi_dot
        2.0;    % theta_dot
        1.0     % psi_dot
    ]);
end