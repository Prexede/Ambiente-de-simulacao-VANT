function controllersConfig = EstimateControllerGains(controllersConfig, quadConfig, specs)
% EstimateControllerGains
% -------------------------------------------------------------------------
% Funcao opcional mantida separada do fluxo principal.
% Esta versao cria ganhos PD aproximados a partir de especificacoes simples.
%
% specs.position.wn, specs.position.zeta
% specs.attitude.wn, specs.attitude.zeta
% -------------------------------------------------------------------------

    if nargin < 3 || isempty(specs)
        error('Informe specs.position e specs.attitude para estimar ganhos.');
    end

    mass = quadConfig.mass;
    I = quadConfig.inertia;

    wnPos = specs.position.wn(:);
    zetaPos = specs.position.zeta(:);
    if numel(wnPos) == 1, wnPos = wnPos*ones(3,1); end
    if numel(zetaPos) == 1, zetaPos = zetaPos*ones(3,1); end

    wnAtt = specs.attitude.wn(:);
    zetaAtt = specs.attitude.zeta(:);
    if numel(wnAtt) == 1, wnAtt = wnAtt*ones(3,1); end
    if numel(zetaAtt) == 1, zetaAtt = zetaAtt*ones(3,1); end

    controllersConfig.position.gains.PD.Kp = diag(wnPos.^2);
    controllersConfig.position.gains.PD.Kd = diag(2*zetaPos.*wnPos);

    inertiaDiag = diag(I);
    controllersConfig.attitude.gains.PD.Kp = diag(inertiaDiag.*wnAtt.^2);
    controllersConfig.attitude.gains.PD.Kd = diag(2*zetaAtt.*wnAtt.*inertiaDiag);

    controllersConfig.gainSource = "estimated";
end
