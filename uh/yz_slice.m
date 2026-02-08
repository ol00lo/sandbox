function yz_slice(G, reports, nx, ny, nz, hy, x_coords, z_coords, field)
% Построение вертикального среза y = const для нескольких моментов времени
% в один ряд (subplot 1x4)
%
% reports - cell array с результатами simulateScheduleAD
% field   - 's' для насыщенности, 'p' для давления

    % Моменты времени для субплотов (доли от начала до конца)
    times_frac = [0.33, 0.66, 1];  % начало, 1/3, 2/3, конец
    nplots = numel(times_frac);

    % Плоскость среза
    y_slice = hy * (floor(ny/2) + 0.5);
    tol = hy / 1000;

    % Ячейки среза
    cells_slice = find(abs(G.cells.centroids(:,2) - y_slice) < tol);

    % Сетка для построения
    [x, z] = meshgrid(x_coords, z_coords);

    % Создаём figure
    figure('Position',[100,100,1400,400]); % широкий график для 4 субплотов в ряд

    for i = 1:nplots
        % Определяем индекс шага
        step_idx = max(1, round(times_frac(i) * numel(reports)));
        step_idx = min(step_idx, numel(reports));

        % Выбор поля
        switch field
            case 's'
                data = reshape(reports{step_idx}.s(cells_slice), [nx, nz]);
                title_str = 'Water Saturation';
                cmap = readmatrix("map_saturation.txt");
            case 'p'
                data = reshape(reports{step_idx}.pressure(cells_slice)/barsa(), [nx, nz]);
                title_str = 'Pressure [atm]';
                cmap = readmatrix("map_pressure.txt");
            otherwise
                error('Unknown field type');
        end

        % Субплот в один ряд
        subplot(1,nplots,i);
        contourf(x, z, transpose(data), 20);
        colormap(cmap);
        colorbar;
        set(gca,'YDir','normal');
        xlabel('X [m]');
        ylabel('Z [m]');
        tlabel = sprintf('%s, step %d/%d', title_str, step_idx, numel(reports));
        title(tlabel);
    end
end
