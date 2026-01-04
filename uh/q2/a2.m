mrstModule add incomp mimetic ad-core mrst-gui

%% СЕТКА
nx = 40; ny = 20; nz = 6;
Lx = 400; Ly = 200; Lz = 30;

hx = Lx/nx; hy = Ly/ny; hz = Lz/nz;

G = cartGrid([nx, ny, nz], [Lx, Ly, Lz]);
G = computeGeometry(G);

x_coords = unique(G.cells.centroids(:,1));
y_coords = unique(G.cells.centroids(:,2));
z_coords = unique(G.cells.centroids(:,3));

assert(numel(x_coords) == nx, 'Ошибка: количество x_coords != nx');
assert(numel(y_coords) == ny, 'Ошибка: количество y_coords != ny');
assert(numel(z_coords) == nz, 'Ошибка: количество z_coords != nz');

%% ПОРОДА
rock.perm = ones(G.cells.num,1) * darcy();
rock.poro = 0.1 * ones(G.cells.num,1);

PV = sum(rock.poro .* G.cells.volumes);
%% ФЛЮИД
mu_w = 1*centi*poise;     
mu_o = 0.5*centi*poise;

fluid = initSimpleFluid( ...
    'mu',  [mu_w, mu_o], ...
    'rho', [1000, 800], ...
    'n',   [2, 2]);

bl = @(s)( s.^2./(s.^2 + (mu_o/mu_w)*(1-s).^2));
%% НАЧАЛЬНОЕ СОСТОЯНИЕ
resSol = initResSol(G, 100*barsa(), [0, 1]);
%% ГРАВИТАЦИЯ
gravity off
%% СКВАЖИНЫ
W = [];

% Добывающая (100, 100)
cells_prod = find_column_at_point(G, [100, 100]);
W = addWell(W, G, rock, cells_prod, ...
    'Type', 'bhp', ...
    'Val', 90*barsa(), ...
    'Radius', 0.1, ...
    'Comp_i', [1, 0], ...
    'Name', 'PROD');

% Нагнетательная (300, 100)
cells_inj = find_column_at_point(G, [300, 100]);
W = addWell(W, G, rock, cells_inj, ...
    'Type', 'bhp', ...
    'Val', 110*barsa(), ...
    'Radius', 0.1, ...
    'Comp_i', [1, 0], ...
    'Name', 'INJ');

prod = strcmp({W.name}, 'PROD');
inj  = strcmp({W.name}, 'INJ');
%% МИМЕТИЧЕСКИЙ ОПЕРАТОР
S = computeMimeticIP(G, rock);
%% СРЕЗ y = const (точно по центру)
y_slice = hy*(floor(ny/2) + 0.5);
tol = hy/1000;

cells_y = abs(G.cells.centroids(:,2) - y_slice) < tol;
assert(sum(cells_y) == nx*nz, 'Ошибка в выборе среза y');
%% ПОДГОТОВКА ФИГУР
figure(5); clf; grid on; title('Well rates')
figure(6); clf; grid on; title('Cumulative well rates')
figure(7); clf; grid on; title('Water cut (producer)')
%% ВРЕМЕННЫЕ МАССИВЫ
time_vec = [];
q_prod = []; q_inj = [];
Q_prod = []; Q_inj = [];
wc_prod = [];
%%  ВРЕМЕННОЙ ЦИКЛ
T  = 400*day();
dt = 20*day();
nstep = ceil(T/dt);

figureplot_dt = 20*day();
next_plot_time = figureplot_dt;
time = 0;

for it = 1:nstep

    % Давление
    resSol = incompMimetic(resSol, G, S, fluid, 'wells', W);

    % Транспорт
    resSol = implicitTransport(resSol, G, dt, rock, fluid, 'wells', W);

    time = time + dt;
    time_vec(end+1) = time/day();

    % Расходы
    qP = resSol.wellSol(prod).flux;
    qI = resSol.wellSol(inj).flux;

    q_prod(end+1) = sum(qP);
    q_inj(end+1)  = sum(qI);

    if numel(time_vec) > 1
        Q_prod(end+1) = trapz(time_vec*day(), q_prod);
        Q_inj(end+1)  = trapz(time_vec*day(), q_inj);
    else
        Q_prod(end+1) = q_prod(end);
        Q_inj(end+1)  = q_inj(end);
    end

    % Фазовые расходы
    qWs = bl(resSol.s(W(prod).cells,1)) .* qP;
    wc_prod(end+1) = sum(qWs)/sum(qP);

    % Графики времени
    figure(5)
    plot(time_vec, q_prod, 'b', time_vec, q_inj, 'r')
    xlabel('Time, days'); ylabel('Rate, m^3/s'); title('Well rates')
    legend('producer','injector'); grid on

    figure(6)
    plot(time_vec, Q_prod, 'b', time_vec, Q_inj, 'r')
    xlabel('Time, days'); ylabel('Cumulative volume, m^3'); title('Cumulative well rates')
    legend('producer','injector'); grid on

    figure(7)
    plot(time_vec, wc_prod, 'k')
    xlabel('Time, days'); ylabel('Water cut'); title('Water cut (producer)'); grid on

    % ОБНОВЛЕНИЕ ПОЛЕЙ каждые figureplot_dt
    if time >= next_plot_time - 1e-8

        % 3D поля
        figure(11)
        plotCellData(G, resSol.pressure/barsa()); colorbar
        xlabel('x, m'); ylabel('y, m'); zlabel('z, m'); title('Pressure field, bar')
        view(45,30)

        figure(12)
        plotCellData(G, resSol.s(:,1)); colorbar
        xlabel('x, m'); ylabel('y, m'); zlabel('z, m'); title('Water saturation field')
        view(45,30)

        % Осреднение по толщине
        P_3D = reshape(resSol.pressure/barsa(), [nx, ny, nz]);
        P_2D = mean(P_3D, 3);

        M_3D = reshape(rock.poro, [nx, ny, nz]);
        M_2D = mean(M_3D, 3);

        S_3D = reshape(resSol.s(:,1), [nx, ny, nz]);
        S_2D = mean(S_3D.*M_3D,3)./M_2D;

        figure(13)
        [x,y] = meshgrid(x_coords, y_coords);
        contourf(x, y, transpose(P_2D)); colorbar
        xlabel('x, m'); ylabel('y, m'); title('Thickness-averaged pressure')

        figure(14)
        contourf(x, y, transpose(S_2D)); colorbar
        xlabel('x, m'); ylabel('y, m'); title('Thickness-averaged water saturation')

        % Срез y=const
        s_mat = reshape(resSol.s(cells_y,1), [nx, nz]);
        p_mat = reshape(resSol.pressure(cells_y)/barsa(), [nx, nz]);
        [x,z] = meshgrid(x_coords, z_coords);

        figure(1)
        contourf(x, z, transpose(s_mat)); colormap(jet); colorbar
        xlabel('x, m'); ylabel('z, m'); title('Water saturation s_w(x,z)')

        figure(2)
        contourf(x, z, transpose(p_mat)); colormap(jet); colorbar
        xlabel('x, m'); ylabel('z, m'); title('Pressure p(x,z), bar')

        % Средние по центру
        sw_avg = S_2D(:, floor(ny/2));
        p_avg  = P_2D(:, floor(ny/2));

        figure(3)
        plot(x_coords, sw_avg,'LineWidth',1.5)
        xlabel('x, m'); ylabel('s_w'); title('Thickness-averaged water saturation'); grid on

        figure(4)
        plot(x_coords, p_avg,'LineWidth',1.5)
        xlabel('x, m'); ylabel('p, bar'); title('Thickness-averaged pressure'); grid on

        % Вертикальные профили расходов
        qP  = resSol.wellSol(inj).flux;
        Sw  = resSol.s(W(inj).cells, 1);
        qWs = bl(Sw) .* qP;
        qOs = qP - qWs;
        z_well = G.cells.centroids(W(inj).cells, 3);

        figure(8)
        plot(qWs, z_well, 'b', qOs, z_well, 'r')
        xlabel('Rate, m^3/s'); ylabel('z, m'); title('Vertical profile of water and oil rates')
        legend('water','oil'); grid on

        next_plot_time = next_plot_time + figureplot_dt;
    end

    % Остановка по закачке 3 PV
    if Q_inj(end) > 3*PV
        t_days  = time / day();
        t_years = t_days / 365;
        fprintf(['Достигнуто 3 PV закачки\n' ...
                 'Время с начала расчёта: %.1f суток (%.2f лет)\n'], ...
                 t_days, t_years);
        break
    end

    drawnow
end
%% FIND_COLUMN_AT_POINT
function cells = find_column_at_point(G, xy)
x = xy(1); y = xy(2);
xc = G.cells.centroids(:,1);
yc = G.cells.centroids(:,2);

hx = min(diff(unique(xc))); hy = min(diff(unique(yc)));
tol = 0.5*min(hx, hy) + 1e-8;

cells = find( abs(xc - x) < tol & abs(yc - y) < tol );
assert(~isempty(cells), 'find_column_at_point: колонка не найдена');
end