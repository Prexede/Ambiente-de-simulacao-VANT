function trajectory = GenerateTrajectory(simConfig)
% GenerateTrajectory
% -------------------------------------------------------------------------
% Gera referencia ponto a ponto usando polinomios de quinta ordem.
% A saida usa matrizes 3xN para facilitar uso no loop.
% -------------------------------------------------------------------------

    t = simConfig.time.t(:).';
    N = numel(t);

    waypoints = simConfig.trajectory.waypoints;
    numSegmentsPerLap = simConfig.trajectory.numSegmentsPerLap;
    segmentTime = simConfig.time.segmentTime;
    totalSegments = simConfig.time.totalSegments;
    yawDesired = simConfig.trajectory.yawDesired;

    r = zeros(3, N);
    v = zeros(3, N);
    a = zeros(3, N);
    psi = yawDesired*ones(1, N);

    lapIndex = zeros(1, N);
    segmentGlobal = zeros(1, N);
    segmentInLap = zeros(1, N);
    segmentLocalTime = zeros(1, N);

    for k = 1:N
        tNow = t(k);

        globalSegment = floor(tNow/segmentTime) + 1;
        globalSegment = min(max(globalSegment, 1), totalSegments);

        localTime = tNow - (globalSegment - 1)*segmentTime;
        localTime = Saturate(localTime, 0, segmentTime);

        localSegment = mod(globalSegment - 1, numSegmentsPerLap) + 1;
        lap = floor((globalSegment - 1)/numSegmentsPerLap) + 1;
        lap = min(lap, simConfig.time.repetitions);

        p0 = waypoints(localSegment, :).';
        pf = waypoints(localSegment + 1, :).';

        for axisIndex = 1:3
            [q, qd, qdd] = QuinticSegment(p0(axisIndex), pf(axisIndex), segmentTime, localTime);
            r(axisIndex, k) = q;
            v(axisIndex, k) = qd;
            a(axisIndex, k) = qdd;
        end

        lapIndex(k) = lap;
        segmentGlobal(k) = globalSegment;
        segmentInLap(k) = localSegment;
        segmentLocalTime(k) = localTime;
    end

    trajectory = struct();
    trajectory.ref.r = r;
    trajectory.ref.v = v;
    trajectory.ref.a = a;
    trajectory.ref.psi = psi;

    trajectory.index.lap = lapIndex;
    trajectory.index.segment = segmentGlobal;
    trajectory.index.segmentInLap = segmentInLap;
    trajectory.index.segmentLocalTime = segmentLocalTime;

    trajectory.waypoints.base = waypoints;
end

function [q, qd, qdd] = QuinticSegment(q0, qf, tf, t)
    a0 = q0;
    a1 = 0;
    a2 = 0;
    a3 = 10*(qf - q0)/tf^3;
    a4 = -15*(qf - q0)/tf^4;
    a5 = 6*(qf - q0)/tf^5;

    q = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
    qd = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
    qdd = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3;
end
