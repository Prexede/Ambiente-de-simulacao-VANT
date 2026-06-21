function condition = GetConditionAtStep(flightPlan, k)
% GetConditionAtStep
% -------------------------------------------------------------------------
% Retorna as condicoes de voo/disturbios na amostra k.
% -------------------------------------------------------------------------

    condition.mass = flightPlan.condition.mass(k);
    condition.windInertial = flightPlan.condition.windInertial(:, k);
    condition.forceInertial = flightPlan.condition.forceInertial(:, k);

    condition.lap = flightPlan.index.lap(k);
    condition.segment = flightPlan.index.segment(k);
    condition.segmentInLap = flightPlan.index.segmentInLap(k);
end