function PlotErrors(simData)
% PlotErrors
% -------------------------------------------------------------------------
% Plota erros de posicao e atitude.
% -------------------------------------------------------------------------

    t = simData.t;

    figure('Name', 'Erros de rastreamento');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(t, simData.error.position.');
    grid on;
    ylabel('erro posicao [m]');
    legend('e_x', 'e_y', 'e_z', 'Location', 'best');
    title('Erro de posicao');

    nexttile;
    plot(t, rad2deg(simData.error.attitude.'));
    grid on;
    ylabel('erro atitude [graus]');
    xlabel('tempo [s]');
    legend('e_phi', 'e_theta', 'e_psi', 'Location', 'best');
    title('Erro de atitude');
end
