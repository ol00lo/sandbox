function cell_indices = findColumnAtPoint(G, xc, yc)
    % Получаем уникальные координаты ячеек по x и y
    ux = unique(G.cells.centroids(:,1));
    uy = unique(G.cells.centroids(:,2));

    % Находим ближайшие значения к xc и yc
    [~, ix] = min(abs(ux - xc));
    [~, iy] = min(abs(uy - yc));

    % Точные координаты столбика
    x0 = ux(ix);
    y0 = uy(iy);

    % Логический вектор ячеек в столбике
    is_in_column = abs(G.cells.centroids(:,1) - x0) < 1e-12 & ...
                   abs(G.cells.centroids(:,2) - y0) < 1e-12;

    cell_indices = find(is_in_column);

    % Сортируем по z (снизу вверх)
    [~, sort_idx] = sort(G.cells.centroids(cell_indices,3), 'ascend');
    cell_indices = cell_indices(sort_idx);

    % Проверка
    assert(length(cell_indices) == G.cartDims(3), ...
           'Ошибка: найденное количество ячеек не соответствует числу слоёв nz');
end
