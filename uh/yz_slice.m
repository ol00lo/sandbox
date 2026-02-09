function yz_slice(G, reports, nx, ny, nz, hy, x_coords, z_coords, field, iPVs)
% Построение вертикального среза y = const для нескольких моментов времени

    step_ids = [iPVs(:).' , numel(reports)];
    nplots = numel(step_ids);
    pv_labels = {'0.5 PV', '1 PV', '2 PV', '3 PV'};

    % Плоскость среза
    y_slice = hy * (floor(ny/2) + 0.5);
    tol = hy / 1000;

    % Ячейки среза
    cells_slice = find(abs(G.cells.centroids(:,2) - y_slice) < tol);

    % Сетка для построения
    [x, z] = meshgrid(x_coords, z_coords);

    figure('Position',[100,100,1000,700]);
    tiledlayout(2,2,'TileSpacing','Compact','Padding','Compact');
    for i = 1:nplots
        step_idx = step_ids(i);

        switch field
            case 's'
                data = reshape(reports{step_idx}.s(cells_slice), [nx, nz]);
                clim([0 1]);
                title_str = 'Water Saturation';
                cmap = readmatrix("map_saturation.txt");
            case 'p'
                clim([90 110]);
                data = reshape(reports{step_idx}.pressure(cells_slice)/barsa(), [nx, nz]);
                title_str = 'Pressure [atm]';
                cmap = readmatrix("map_pressure.txt");
            otherwise
                error('Unknown field type');
        end
        
        nexttile;
        shading interp;
        contourf(x, z, transpose(data), 20);
        colormap(cmap);
        colorbar;
        set(gca,'YDir','normal');
        xlabel('X [m]');
        ylabel('Z [m]');
        title(sprintf('%s – %s', title_str, pv_labels{i}));
    end
end
