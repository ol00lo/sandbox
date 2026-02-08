function fields2d_xy(reports, rock, nx, ny, nz, x_coords, y_coords, field, iPV1, iPV2)
% Визуализация толщинно-усреднённых 2D полей (x–y)

    step_ids = [iPV1, iPV2, numel(reports)];
    nplots = numel(step_ids);
    pv_labels = {'1 PV', '2 PV', '3 PV'};
    
    figure('Position',[100,100,1800,500]);
    tiledlayout(1,nplots,'TileSpacing','Compact','Padding','Compact');
    s_x = {};
    for i = 1:nplots
        step_idx = step_ids(i);

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
        title(sprintf('%s – %s', title_str, pv_labels{i}));
    end

    if field == "s"
        figure;
        for i = 1:nplots
            plot(x_coords, s_x{i}, 'LineWidth', 1.5); hold on;
            xlabel('x, m');
            ylabel('water saturation');
        end
        legend(pv_labels, 'Location', 'best');
        grid on;
    end
end
