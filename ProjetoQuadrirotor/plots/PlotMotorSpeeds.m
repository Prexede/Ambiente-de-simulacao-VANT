function PlotMotorSpeeds(simData)
% PlotMotorSpeeds
% -------------------------------------------------------------------------
% Plota velocidades dos motores e indica saturacao.
% -------------------------------------------------------------------------

    t = simData.t;

    figure('Name', 'Motores');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(t, simData.cmd.motorOmega.');
    grid on;
    ylabel('omega [rad/s]');
    legend('omega_1', 'omega_2', 'omega_3', 'omega_4', 'Location', 'best');
    title('Velocidade dos motores');

    nexttile;
    stairs(t, simData.diagnostic.anyMotorSaturation, 'LineWidth', 1.2);
    grid on;
    ylim([-0.1 1.1]);
    ylabel('saturacao');
    xlabel('tempo [s]');
    title('Saturacao de motor');
end
