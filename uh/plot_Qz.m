function plot_Qz(G, WP, reports)

    z = G.cells.centroids(WP.cells, 3);
    % Моменты времени для субплотов (доли от начала до конца)
    times_frac = [0.33, 0.66, 1]; % три графика
    nplots = numel(times_frac);

    % Создаём широкую фигуру
    figure;
    t = tiledlayout(1,nplots,'TileSpacing','Compact','Padding','Compact');

    for i = 1:nplots
        % Индекс шага
        step_idx = max(1, round(times_frac(i) * numel(reports)));
        step_idx = min(step_idx, numel(reports));

        % Субплот
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
        xlim([0 250]);
        tlabel = sprintf('Rate(z), step %d/%d', step_idx, numel(reports));
        title(tlabel);
    end
    
end