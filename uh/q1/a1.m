mrstModule add ad-core ad-blackoil incomp mimetic

%% СЕТКА
nx = 20; ny = 20; nz = 10;
Lx = 100; Ly = 100; Lz = 20;

G = cartGrid([nx, ny, nz], [Lx, Ly, Lz]);
G = computeGeometry(G);

% figure; plotGrid(G); view(3); axis tight
% title('Grid')
%% Центральное сечение (по x и y)
xc = Lx/2; 
yc = Ly/2;

cells_center_slice = find_column_at_point(G, xc, yc);
z_slice = G.cells.centroids(cells_center_slice,3);

% Верхняя ячейка столбика (для мониторинга насыщенности нефти наверху)
cell_upper = cells_center_slice(end);
%% ПОРОДА
rock.perm = 100*milli*darcy()*ones(G.cells.num,1);
rock.poro = 0.3*ones(G.cells.num,1);
%% ФЛЮИД
fluid = initSimpleFluid( ...
    'mu'  , [1, 5]*centi*poise, ...      % вода / нефть
    'rho' , [1000, 800], ...             % плотности
    'n'   , [2, 2]);                     % Corey
%% НАЧАЛЬНОЕ СОСТОЯНИЕ
resSol = initResSol(G, 100*barsa(), [0, 1]); % по умолчанию нефть

z = G.cells.centroids(:,3);
resSol.s(:,2) = z < Lz/2;   % нефть внизу
resSol.s(:,1) = 1 - resSol.s(:,2);
%% ГРАНИЧНЫЕ УСЛОВИЯ
bc = [];
bc = pside(bc, G, 'TOP', 100*barsa());
%% ГРАВИТАЦИЯ
gravity on
gravity reset on
gravity([0 0 -9.81]);
%% МИМЕТИЧЕСКИЙ ОПЕРАТОР
S = computeMimeticIP(G, rock);
%% ВРЕМЕННОЙ ЦИКЛ
figure(10); hold on; grid on
xlabel('Water saturation s_1')
ylabel('z')
title('s_1(z) in central section')

figure(11); hold on; grid on
xlabel('Pressure [bar]')
ylabel('z')
title('p(z) in central section')

figure(12); hold on; grid on
xlabel('Time [days]')
ylabel('s_1 (top cell)')
title('s_1(t) at domain top')

time_vec = [];
s_upper_vec = [];

T  = 50*year();
dt = 100*day();
nstep = ceil(T/dt);

plot_dt = 1*year();
next_plot_time = plot_dt;

time = 0;

for it = 1:nstep

    % Давление
    resSol = incompMimetic(resSol, G, S, fluid, 'bc', bc);

    % Транспорт
    resSol = implicitTransport(resSol, G, dt, rock, fluid);

    time = time + dt;

    % ---- Графики каждые 20 дней ----
    if time >= next_plot_time - 1e-6

        % s(z)
        figure(10)
        plot(z_slice, resSol.s(cells_center_slice,1), ...
            'LineWidth', 1.2, ...
            'DisplayName', ['t = ', num2str(time/day()), ' d'])

        % p(z)
        figure(11)
        plot(z_slice, convertTo(resSol.pressure(cells_center_slice), barsa()), ...
             'LineWidth', 1.2, ...
             'DisplayName', ['t = ', num2str(time/day()), ' d'])

        next_plot_time = next_plot_time + plot_dt;
    end

    % ---- s1 в верхней ячейке столбика ----
    time_vec(end+1) = time/day();
    s_upper_vec(end+1) = resSol.s(cell_upper,1);

    figure(12)
    plot(time_vec, s_upper_vec, 'k-', 'LineWidth', 1.5)
    drawnow
end
%% FIND_COLUMN_AT_POINT
function cell_indices = find_column_at_point(G, xc, yc)
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
