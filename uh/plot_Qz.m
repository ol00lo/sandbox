function plot_Qz(G, WP, reports, iPVs)

    z = G.cells.centroids(WP.cells, 3);
    %xWP = 3*400/4; yWP = 200/2;
    %z = G.cells.centroids(findColumnAtPoint(G, xWP, yWP), 3);
    step_ids = [iPVs(:).' , numel(reports)];
    nplots = numel(step_ids);
    pv_labels = {'0.5 PV', '1 PV', '2 PV', '3 PV'};

    figure;
    tiledlayout(1,nplots,'TileSpacing','Compact','Padding','Compact');

    for i = 1:nplots
        step_idx = step_ids(i);

        nexttile;
        ws = reports{step_idx};
        qw = -ws.wellSol(2).flux(:, 1)*day;
        qo = -ws.wellSol(2).flux(:, 2)*day;
        q = qo + qw;
        plot(qo, z,  'c-', 'LineWidth', 1.5); hold on;
        plot(qw, z,  'k-', 'LineWidth', 1.5);
        plot(q, z, 'r-', 'LineWidth', 1.5);
        xlabel("q, m3/day");
        ylabel("z, m");
        legend("q_{oil}", "q_{water}", "q_{total}")
        xlim([0 490]);
        tlabel = sprintf('Rate(z), %s', pv_labels{i});
        title(tlabel);
    end
end