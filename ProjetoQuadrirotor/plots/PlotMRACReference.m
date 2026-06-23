function PlotMRACReference(simData)
% PlotMRACReference
% -------------------------------------------------------------------------
% Plota comando, modelo de referencia MRAC e resposta real.
% Esta figura so e usada quando o modelo MRAC_Test gera simData.mrac.
% -------------------------------------------------------------------------

    t = simData.t;

    figure('Name', 'MRAC - modelo de referencia');
    tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(t, simData.ref.r(3,:), '--');
    hold on;
    plot(t, simData.mrac.altitude.referenceState(1,:), '-.');
    plot(t, simData.state.x(3,:), '-');
    grid on;
    ylabel('z [m]');
    legend('z_{cmd}', 'z_{ref}', 'z', 'Location', 'best');
    title('Altitude');

    attitudeNames = {'phi', 'theta', 'psi'};

    for i = 1:3
        nexttile;
        plot(t, rad2deg(simData.cmd.attitudeDesired(i,:)), '--');
        hold on;
        plot(t, rad2deg(squeeze(simData.mrac.attitude.referenceState(1,i,:))).', '-.');
        plot(t, rad2deg(simData.state.x(6+i,:)), '-');
        grid on;
        ylabel([attitudeNames{i}, ' [graus]']);
        legend([attitudeNames{i}, '_{cmd}'], [attitudeNames{i}, '_{ref}'], attitudeNames{i}, 'Location', 'best');

        if i == 3
            xlabel('tempo [s]');
        end
    end
end
