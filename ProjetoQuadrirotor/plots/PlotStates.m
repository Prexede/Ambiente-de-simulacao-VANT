function PlotStates(simData)
% PlotStates
% -------------------------------------------------------------------------
% Plota posicao, velocidade e atitude no tempo.
% -------------------------------------------------------------------------

    t = simData.t;
    x = simData.state.x;

    figure('Name', 'Estados translacionais');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(t, x(1:3, :).');
    grid on;
    ylabel('posicao [m]');
    legend('x', 'y', 'z', 'Location', 'best');
    title('Posicao');

    nexttile;
    plot(t, x(4:6, :).');
    grid on;
    ylabel('velocidade [m/s]');
    legend('x_dot', 'y_dot', 'z_dot', 'Location', 'best');
    title('Velocidade');

    nexttile;
    plot(t, rad2deg(x(7:9, :).'));
    grid on;
    ylabel('atitude [graus]');
    xlabel('tempo [s]');
    legend('phi', 'theta', 'psi', 'Location', 'best');
    title('Atitude');
end
