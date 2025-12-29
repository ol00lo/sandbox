mrstModule add incomp mimetic ad-core mrst-gui

%% ------------------------
% СЕТКА
%% ------------------------
nx = 40; ny = 20; nz = 6;
Lx = 400; Ly = 200; Lz = 30;

G = cartGrid([nx, ny, nz], [Lx, Ly, Lz]);
G = computeGeometry(G);

%% ------------------------
% ПОРОДА
%% ------------------------
rock.perm = ones(G.cells.num,1) * darcy();
rock.poro = 0.1 * ones(G.cells.num,1);

%% ------------------------
% ФЛЮИД
%% ------------------------
mu_w = 1;                 % вода
mu_o = 0.5;               % нефть

fluid = initSimpleFluid( ...
    'mu',  [mu_w, mu_o]*centi*poise, ...
    'rho', [1000, 800], ...
    'n',   [2, 2]);       % Corey n=2
bl = @(s)( s.^2./(s.^2 + (mu_o/mu_w)*(1-s).^2));

%% ------------------------
% НАЧАЛЬНОЕ СОСТОЯНИЕ
%% ------------------------
resSol = initResSol(G, 100*barsa(), [0, 1]); % пласт заполнен нефтью

%% ------------------------
% ГРАВИТАЦИЯ
%% ------------------------
gravity off

%% ------------------------
% СКВАЖИНЫ
%% ------------------------
W = [];

% Добывающая (100,100)
W = addWell(W, G, rock, ...
    find( ...
        abs(G.cells.centroids(:,1)-105) < 1 & ...
        abs(G.cells.centroids(:,2)-105) < 1 ), ...
    'Type', 'bhp', ...
    'Val', 10*barsa(), ...
    'Radius', 0.1, ...
    'Comp_i', [1, 0], ...
    'Name', 'PROD');

% Нагнетательная (300,100)
W = addWell(W, G, rock, ...
    find( ...
        abs(G.cells.centroids(:,1)-305) < 1 & ...
        abs(G.cells.centroids(:,2)-105) < 1 ), ...
    'Type', 'bhp', ...
    'Val', 500*barsa(), ...
    'Radius', 0.1, ...
    'Comp_i', [1, 0], ...
    'Name', 'INJ');

%% ------------------------
% МИМЕТИЧЕСКИЙ ОПЕРАТОР
%% ------------------------
S = computeMimeticIP(G, rock);

%% ------------------------
% Подготовка индексов
y_slice = 100;
tol = Ly/ny;

cells_y100 = abs(G.cells.centroids(:,2) - y_slice) < tol;

x_slice = G.cells.centroids(cells_y100,1);
z_slice = G.cells.centroids(cells_y100,3);

[x_unique, ~, ix] = unique(G.cells.centroids(cells_y100,1));
prod = strcmp({W.name}, 'PROD');
inj  = strcmp({W.name}, 'INJ');

%%
% Подготовка фигур
figure(1); clf; hold on; grid on
title('s_w(x,z) at y=100')

figure(2); clf; hold on; grid on
title('p(x,z) at y=100')

figure(3); clf; hold on; grid on
title('Thickness-averaged s_w(x)')

figure(4); clf; hold on; grid on
title('Thickness-averaged p(x)')

figure(5); clf; grid on
title('Well rates')

figure(6); clf; grid on
title('Cumulative well rates')

figure(7); clf; grid on
title('Water cut (producer)')

figure(8); clf; hold on; grid on
title('q(z), q_oil(z) at injector')

time_vec = [];
q_prod = []; q_inj = [];
Q_prod = []; Q_inj = [];
wc_prod = [];

%% vtk
vtk_folder = 'vtk_output';
if ~exist(vtk_folder, 'dir')
    mkdir(vtk_folder);
end

%% ------------------------
% ВРЕМЕННОЙ ЦИКЛ
%% ------------------------
T  = 200*day();
dt = 5*day();
nstep = ceil(T/dt);

figureplot_dt = 20*day();
next_plot_time = plot_dt;
time = 0;

for it = 1:nstep
    % --- давление ---
    resSol = incompMimetic(resSol, G, S, fluid, 'wells', W);

    % --- транспорт ---
    resSol = implicitTransport(resSol, G, dt, rock, fluid, 'wells', W);

    time = time + dt;

    if true
    % ========== 3. ЗАВИСЯЩИЕ ОТ ВРЕМЕНИ ГРАФИКИ ==========
    time_vec(end+1) = time/day();

    % расходы
    qP = resSol.wellSol(prod).flux
    qI = resSol.wellSol(inj).flux

    q_prod(end+1) = sum(qP);
    q_inj(end+1)  = sum(qI);

    if length(time_vec) > 1
        Q_prod(end+1) = trapz(time_vec, q_prod);
        Q_inj(end+1)  = trapz(time_vec, q_inj);
    else
        Q_prod(end+1) = q_prod;
        Q_inj(end+1) = q_inj;
    end

    qWs = bl(resSol.s(W(prod).cells, 1)) .* qP;
    qOs = qP - qWs; 

    % обводнённость
    wc_prod(end+1) = sum(qWs)/sum(qP);

    % --- графики ---
    figure(5)
    plot(time_vec, q_prod, 'b', time_vec, q_inj, 'r')
    title('Well rates')
    legend("producer", "injector")
    
    figure(6)
    plot(time_vec, Q_prod, 'b', time_vec, Q_inj, 'r')
    title('Cumulative well rates')
    legend("producer", "injector")
    
    figure(7)
    plot(time_vec, wc_prod, 'k')
    title('Water cut (producer)')

    % ========== ОБНОВЛЕНИЕ КАЖДЫЕ 20 ДНЕЙ ==========
    if false && time >= next_plot_time - 1e-6

        % ---- 1. Вертикальный срез ----
        figure(1)
        scatter(x_slice, z_slice, 40, resSol.s(cells_y100,1), 'filled')

        figure(2)
        scatter(x_slice, z_slice, 40, ...
            convertTo(resSol.pressure(cells_y100), barsa()), 'filled')

        % ---- 2. Осреднение по z ----
        sw_avg = accumarray(ix, resSol.s(cells_y100,1), [], @mean);
        p_avg  = accumarray(ix, ...
            convertTo(resSol.pressure(cells_y100), barsa()), [], @mean);

        figure(3)
        plot(x_unique, sw_avg, 'LineWidth', 1.5)

        figure(4)
        plot(x_unique, p_avg, 'LineWidth', 1.5)

        % ---- 4. q(z) для нагнетательной ----
        cells_inj = W(inj).cells;
        z_well = G.cells.centroids(cells_inj,3);

        q_w = resSol.wellSol(inj).qWs / numel(z_well);
        q_o = resSol.wellSol(inj).qOs / numel(z_well);

        figure(8)
        plot(q_w*ones(size(z_well)), z_well, 'b', ...
             q_o*ones(size(z_well)), z_well, 'r')

        % ---- 5. s(x) при y=100 ----
        figure(9)
        plot(x_unique, sw_avg, 'LineWidth', 1.5)

        next_plot_time = next_plot_time + plot_dt;
    end
    end
    drawnow
end
