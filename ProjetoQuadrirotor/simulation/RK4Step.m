function stateNext = RK4Step(state, motorOmega, quadConfig, disturbance, Ts)
% RK4Step
% -------------------------------------------------------------------------
% Integracao por Runge-Kutta de quarta ordem.
% O comando dos motores e considerado constante durante o passo.
% -------------------------------------------------------------------------

    k1 = QuadrotorDynamics(state, motorOmega, quadConfig, disturbance);
    k2 = QuadrotorDynamics(state + 0.5*Ts*k1, motorOmega, quadConfig, disturbance);
    k3 = QuadrotorDynamics(state + 0.5*Ts*k2, motorOmega, quadConfig, disturbance);
    k4 = QuadrotorDynamics(state + Ts*k3, motorOmega, quadConfig, disturbance);

    stateNext = state + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end
