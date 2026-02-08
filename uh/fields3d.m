function fields3d(G, reports, Lx, Ly, hy, field)
% Визуализация 3D поля давления или насыщенности

    % Моменты времени для субплотов (доли от начала до конца)
    times_frac = [0.33, 0.66, 1]; % три графика
    nplots = numel(times_frac);

    % Индексы для отображения (как в твоей оригинальной функции)
    indices3d = ( ...
        G.cells.centroids(:,1) > Lx/2 | ...
        G.cells.centroids(:,2) + hy > Ly/2 );

    % Создаём широкую фигуру
    figure('Position',[100,100,1800,600]);
    t = tiledlayout(1,nplots,'TileSpacing','Compact','Padding','Compact');

    for i = 1:nplots
        % Индекс шага
        step_idx = max(1, round(times_frac(i) * numel(reports)));
        step_idx = min(step_idx, numel(reports));

        % Выбор поля
        switch field
            case 'p'
                data = reports{step_idx}.pressure / barsa();
                title_str = 'Pressure [atm]';
                cmap = readmatrix("map_pressure.txt");
            case 's'
                data = reports{step_idx}.s(:,1);
                title_str = 'Water saturation';
                cmap = readmatrix("map_saturation.txt");
            otherwise
                error('Unknown field type');
        end

        % Субплот
        nexttile;
        shading interp;
        plotCellData(G, data, indices3d);
        colormap(cmap);
        colorbar;
        xlabel('X'); ylabel('Y'); zlabel('Z');
        view(-45,30);

        tlabel = sprintf('%s, step %d/%d', title_str, step_idx, numel(reports));
        title(tlabel);
    end
end
