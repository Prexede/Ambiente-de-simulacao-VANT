function stateNext = EulerStep(state, motorOmega, quadConfig, disturbance, Ts)
% EulerStep
% -------------------------------------------------------------------------
% Integracao por Euler explicito.
% -------------------------------------------------------------------------
    stateNext = state + Ts*QuadrotorDynamics(state, motorOmega, quadConfig, disturbance);
end
