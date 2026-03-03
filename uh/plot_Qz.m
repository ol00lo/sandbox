function plot_Qz(G, WP, reports, iPVs)

    z = G.cells.centroids(WP.cells, 3);

    step_ids = [iPVs(:).' , numel(reports)];
    nplots = numel(step_ids);
    pv_labels = {'0.5 PV', '1 PV', '2 PV', '3 PV'};

    figure('Position',[100 100 1000 600]);

    t = tiledlayout(1,nplots,'TileSpacing','compact','Padding','compact');

    for i = 1:nplots
        step_idx = step_ids(i);

        nexttile;

        ws = reports{step_idx};
        qw = -ws.wellSol(2).flux(:, 1)*day;
        qo = -ws.wellSol(2).flux(:, 2)*day;
        q  = qo + qw;

        h1 = plot(qo, z, 'c-', 'LineWidth', 1.5); hold on;
        h2 = plot(qw, z, 'k-', 'LineWidth', 1.5);
        h3 = plot(q,  z, 'r-', 'LineWidth', 1.5);

        xlabel("q, m^3/day");
        ylabel("z, m");
        xlim([0 450]);
        set(gca,'YDir','normal');

        title(sprintf('Rate(z), %s', pv_labels{i}));
    end

    % --- Общая легенда ---
    lgd = legend([h1 h2 h3], ...
        "q_{oil}", "q_{water}", "q_{total}");

    lgd.Layout.Tile = 'east';   % справа от всех графиков
end