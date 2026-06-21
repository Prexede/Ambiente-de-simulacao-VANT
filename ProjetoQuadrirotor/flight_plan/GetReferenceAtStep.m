function ref = GetReferenceAtStep(flightPlan, k)
% GetReferenceAtStep
% -------------------------------------------------------------------------
% Retorna a referencia da trajetoria na amostra k.
% -------------------------------------------------------------------------

    ref.r = flightPlan.ref.r(:, k);
    ref.v = flightPlan.ref.v(:, k);
    ref.a = flightPlan.ref.a(:, k);
    ref.psi = flightPlan.ref.psi(k);
    ref.lap = flightPlan.index.lap(k);
    ref.segment = flightPlan.index.segment(k);
    ref.segmentInLap = flightPlan.index.segmentInLap(k);
end
