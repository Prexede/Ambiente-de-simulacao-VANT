function waypoints = BuildWaypoints(trajectoryType)
% BuildWaypoints
% -------------------------------------------------------------------------
% Retorna os waypoints base de uma volta da trajetoria.
% Cada segmento corresponde ao movimento da linha i para a linha i+1.
% -------------------------------------------------------------------------

    switch lower(string(trajectoryType))
        case "quad"
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0;
               -5.0    0.0   10.0;
               -5.0   -5.0   10.0;
                5.0   -5.0   10.0;
                5.0    5.0   10.0;
               -5.0    5.0   10.0;
               -5.0    0.0   10.0;
                0.0    0.0    0.0
            ];

        case "quad50"
            waypoints = [
                 0.0     0.0    0.0;
                 0.0     0.0   50.0;
               -25.0     0.0   50.0;
               -25.0   -25.0   50.0;
                25.0   -25.0   50.0;
                25.0    25.0   50.0;
               -25.0    25.0   50.0;
               -25.0     0.0   50.0;
                 0.0     0.0    0.0
            ];

        case "testz"
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0
            ];

        case "testx"
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0;
                5.0    0.0   10.0
            ];

        case "testy"
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0;
                0.0    5.0   10.0
            ];

        case "testxy"
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0;
                5.0    0.0   10.0;
                5.0    5.0   10.0
            ];

        case "testxyz"
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0;
                5.0    0.0   10.0;
                5.0    5.0   10.0;
                0.0    0.0    0.0
            ];

        case {"hex", "hexagon"}
            waypoints = [
                0.0    0.0    0.0;
                0.0    0.0   10.0;
                5.0    0.0   10.0;
                2.5    4.33  10.0;
               -2.5    4.33  10.0;
               -5.0    0.0   10.0;
               -2.5   -4.33  10.0;
                2.5   -4.33  10.0;
                5.0    0.0   10.0;
                0.0    0.0    0.0
            ];

        otherwise
            error('Tipo de trajetoria "%s" nao reconhecido.', string(trajectoryType));
    end
end
