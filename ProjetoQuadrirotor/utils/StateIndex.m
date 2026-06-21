function idx = StateIndex()
% StateIndex
% -------------------------------------------------------------------------
% Indices padronizados do vetor de estados:
%   [x y z x_dot y_dot z_dot phi theta psi p q r]'
% -------------------------------------------------------------------------

    idx.position = 1:3;
    idx.velocity = 4:6;
    idx.attitude = 7:9;
    idx.bodyRate = 10:12;

    idx.x = 1;
    idx.y = 2;
    idx.z = 3;

    idx.xDot = 4;
    idx.yDot = 5;
    idx.zDot = 6;

    idx.phi = 7;
    idx.theta = 8;
    idx.psi = 9;

    idx.p = 10;
    idx.q = 11;
    idx.r = 12;
end
