function PlotTrajectory(simData)
% PlotTrajectory
% -------------------------------------------------------------------------
% Compara trajetoria desejada e trajetoria realizada.
% -------------------------------------------------------------------------

    idx = StateIndex();
    r = simData.state.x(idx.position, :);
    rRef = simData.ref.r;

    figure('Name', 'Trajetoria 3D');
    plot3(rRef(1,:), rRef(2,:), rRef(3,:), '--', 'LineWidth', 1.5);
    hold on;
    plot3(r(1,:), r(2,:), r(3,:), 'LineWidth', 1.2);
    grid on;
    axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Trajetoria desejada vs realizada');
    legend('Referencia', 'Quadrotor', 'Location', 'best');
    view(3);
end
