function stateNext = IntegrateStep(state, motorOmega, quadConfig, disturbance, simConfig)
% IntegrateStep
% -------------------------------------------------------------------------
% Seleciona o metodo de integracao configurado.
% -------------------------------------------------------------------------

    method = lower(string(simConfig.integration.method));

    switch method
        case "euler"
            stateNext = EulerStep(state, motorOmega, quadConfig, disturbance, simConfig.time.Ts);
        case "rk4"
            stateNext = RK4Step(state, motorOmega, quadConfig, disturbance, simConfig.time.Ts);
        otherwise
            error('Metodo de integracao "%s" nao implementado. Use "Euler" ou "RK4".', method);
    end

    stateNext(7:9) = WrapAngle(stateNext(7:9));
end
