function plots(G, nx, ny, nz, Lx, Ly, rock, result, W)
    x_coords = unique(G.cells.centroids(:,1));
    y_coords = unique(G.cells.centroids(:,2));
    z_coords = unique(G.cells.centroids(:,3));

    hy = Ly/ny;

    reports = result.reports;
    iPVs    = result.iPVs;
    % 2D Фронт насыщенности и давления между скважинами (1)
    %yz_slice(G, reports, nx, ny, nz, hy, x_coords, z_coords, 's', iPVs);
    %yz_slice(G, reports, nx, ny, nz, hy, x_coords, z_coords, 'p', iPVs);

    % 3D поле насыщенности и давления (2)
    %fields3d(G, reports, Lx, Ly, hy, 's', iPVs);
    %fields3d(G, reports, Lx, Ly, hy, 'p', iPVs);

    % Горизонтально усреднённое поле насыщенности и давления (2)
    %fields2d_xy(reports, rock, nx, ny, nz, x_coords, y_coords, 's', iPVs);
    fields2d_xy(reports, rock, nx, ny, nz, x_coords, y_coords, 'p', iPVs);

    % 3
    %plot_Q(result.time, result.QI, result.QPo, result.wcP, result.qI, result.qP, result.qPo);

    % 4
    %plot_Qz(G, W(2), reports, iPVs);
end