function fields3d(G, reports, Lx, Ly, hy, field, iPVs)
% Визуализация 3D поля давления или насыщенности

    step_ids = [iPVs(:).' , numel(reports)];
    nplots = numel(step_ids);
    pv_labels = {'0.5 PV', '1 PV', '2 PV', '3 PV'};

    % Индексы для отображения (как в твоей оригинальной функции)
    indices3d = ( ...
        G.cells.centroids(:,1) > Lx/2 | ...
        G.cells.centroids(:,2) + hy > Ly/2 );

    figure('Position',[100,100,1000,700]);
    tiledlayout(2,2,'TileSpacing','Compact','Padding','Compact');
    for i = 1:nplots
        step_idx = step_ids(i);

        switch field
            case 'p'
                clim([95 110])
                data = reports{step_idx}.pressure / barsa();
                title_str = 'Pressure [atm]';
                cmap = readmatrix("map_pressure.txt");
            case 's'
                clim ([0 1])
                data = reports{step_idx}.s(:,1);
                title_str = 'Water saturation';
                cmap = readmatrix("map_saturation.txt");
            otherwise
                error('Unknown field type');
        end

        nexttile;
        shading interp;
        plotCellData(G, data, indices3d);
        colormap(cmap);
        colorbar;
        xlabel('X'); ylabel('Y'); zlabel('Z');
        view(-45,30);

        title(sprintf('%s – %s', title_str, pv_labels{i}));
    end
end
