function fields2d_xy(reports, rock, nx, ny, nz, x_coords, y_coords, field)
% Визуализация толщинно-усреднённых 2D полей (x–y)
% Строит 3 субплота в один ряд для разных моментов времени
%
% reports - cell array с результатами simulateScheduleAD
% field   - 's' для насыщенности, 'p' для давления

    % Моменты времени для субплотов (доли от начала до конца)
    times_frac = [0.33, 0.66, 1];
    nplots = numel(times_frac);

    % Создаём широкую фигуру
    figure('Position',[100,100,1800,500]);
    t = tiledlayout(1,nplots,'TileSpacing','Compact','Padding','Compact');
    s_x = {};
    for i = 1:nplots
        % Индекс шага
        step_idx = max(1, round(times_frac(i) * numel(reports)));
        step_idx = min(step_idx, numel(reports));

        % Выбор поля
        switch field
            case 'p'
                P_3D = reshape(reports{step_idx}.pressure, [nx, ny, nz]);
                P_2D = mean(P_3D, 3);
                data = P_2D / barsa();
                title_str = 'Thickness-averaged pressure [atm]';
                cmap = readmatrix("map_pressure.txt");
            case 's'
                % 3D массивы для выбранного шага
                M_3D = reshape(rock.poro, [nx, ny, nz]);
                S_3D = reshape(reports{step_idx}.s(:,1), [nx, ny, nz]);
                % Толщинное осреднение
                M_2D = mean(M_3D, 3);
                S_2D = mean(S_3D .* M_3D, 3) ./ M_2D;
                data = S_2D;
                title_str = 'Thickness-averaged saturation';
                cmap = readmatrix("map_saturation.txt");
                s_x{end+1} = S_2D(:, ceil(size(S_2D, 2)/2));
            otherwise
                error('Unknown field type');
        end

        % Субплот
        nexttile;
        contourf(x_coords, y_coords, transpose(data), 15);
        colormap(cmap);
        colorbar;
        xlabel('x, m');
        ylabel('y, m');
        tlabel = sprintf('%s, step %d/%d', title_str, step_idx, numel(reports));
        title(tlabel);
    end

    if field == "s"
        figure;
        legends = {};
        colors = ["r-", "g-", "b-", "k-", "c-"];
        for i = 1:nplots
            % Индекс шага
            step_idx = max(1, round(times_frac(i) * numel(reports)));
            step_idx = min(step_idx, numel(reports));
  
            % Субплот
            plot(x_coords, s_x{i}, 'LineWidth', 1.5); hold on;
            xlabel('x, m');
            ylabel('water saturation');
            legends{end+1} = sprintf('step %d/%d', step_idx, numel(reports));
            title(tlabel);
        end
        legend(legends);
        grid on;
    end
end
