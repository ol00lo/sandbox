function plots(G, nx, ny, nz, Lx, Ly, rock, result, W)
    x_coords = unique(G.cells.centroids(:,1));
    y_coords = unique(G.cells.centroids(:,2));
    z_coords = unique(G.cells.centroids(:,3));

    hy = Ly/ny;

    reports = result.reports;
    iPV1 = result.iPV1;
    iPV2 = result.iPV2;
    % 2D Фронт насыщенности и давления между скважинами (1)
    yz_slice(G, reports, nx, ny, nz, hy, x_coords, z_coords, 's', iPV1, iPV2);
    yz_slice(G, reports, nx, ny, nz, hy, x_coords, z_coords, 'p', iPV1, iPV2);

    % 3D поле насыщенности и давления (2)
    fields3d(G, reports, Lx, Ly, hy, 's', iPV1, iPV2);
    fields3d(G, reports, Lx, Ly, hy, 'p', iPV1, iPV2);

    % Горизонтально усреднённое поле насыщенности и давления (2)
    fields2d_xy(reports, rock, nx, ny, nz, x_coords, y_coords, 's', iPV1, iPV2);
    fields2d_xy(reports, rock, nx, ny, nz, x_coords, y_coords, 'p', iPV1, iPV2);

    % 3
    plot_Q(result.time, result.QI, result.QPo, result.wcP, result.qI, result.qP, result.qPo);

    % 4
    plot_Qz(G, W(2), reports, iPV1, iPV2);
end