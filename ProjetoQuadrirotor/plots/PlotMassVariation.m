function PlotMassVariation(simData)
% PlotMassVariation
% -------------------------------------------------------------------------
% Plota massa e disturbio de vento no tempo.
% -------------------------------------------------------------------------

    t = simData.t;

    figure('Name', 'Condicoes de voo');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(t, simData.condition.mass, 'LineWidth', 1.2);
    grid on;
    ylabel('massa [kg]');
    title('Massa usada na planta');

    nexttile;
    plot(t, simData.condition.windInertial.');
    grid on;
    ylabel('forca [N]');
    xlabel('tempo [s]');
    legend('F_x', 'F_y', 'F_z', 'Location', 'best');
    title('Forca externa inercial');
end
