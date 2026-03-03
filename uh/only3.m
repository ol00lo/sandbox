clear; close all; clc;
%% МОДУЛИ
mrstModule add incomp
mrstModule add ad-core ad-blackoil ad-props mrst-gui

gravity off
%% ГЕОМЕТРИЯ
nx = 40; ny = nx/2; nz = 3;
Lx = 400; Ly = 200; Lz = 30;
hx = Lx/nx; hy = Ly/ny; hz = Lz/nz;

G = cartGrid([nx, ny, nz], [Lx, Ly, Lz]);
G = computeGeometry(G);
%% ФЛЮИД
Kmu = 0.5;
a = 2;
mu_w = 1*centi*poise;
mu_o = mu_w/Kmu;

fluid = initSimpleFluid( ...
    'phases', 'WO', ...
    'mu',  [mu_w, mu_o], ...
    'rho', [1000, 800], ...
    'n',   [a, a]);
%% НАЧАЛЬНОЕ СОСТОЯНИЕ
p0 = 100*barsa();
state = initState(G, [], p0, [0, 1]);

%% ПЛАСТ
m0 = 0.1;           
k0 = 1*darcy();    
nlayers = 3;
alpha = [1, 0.9, 1.1];
beta  = [1, 0.5, 2];

cellsPerLayer = nz / nlayers;

poro = zeros(nx, ny, nz);
perm = zeros(nx, ny, nz);

for i = 1:nlayers
    k1 = (i-1)*cellsPerLayer + 1;
    k2 = i * cellsPerLayer;

    poro(:,:,k1:k2) = m0 * alpha(i);
    perm(:,:,k1:k2) = k0 * beta(i);
end

rock_het.poro = poro(:);
rock_het.perm = perm(:);

%% СКВАЖИНЫ
xWI = Lx/4;   yWI = Ly/2;
xWP = 3*Lx/4; yWP = Ly/2;

pI = 110*barsa();
pP = 90*barsa();

zeta_WI = [0, 2/3];   % вскрыты нижние 2/3 пласта
zeta_WP = [1/3, 1];   % верхние 2/3

layer_WI = max(1, round(zeta_WI * nz));
layer_WP = max(1, round(zeta_WP * nz));

x_coords = unique(G.cells.centroids(:,1));
y_coords = unique(G.cells.centroids(:,2));
z_coords = unique(G.cells.centroids(:,3));
cells_WI_all = findColumnAtPoint(G, xWI, yWI);
cells_WI = cells_WI_all(G.cells.centroids(cells_WI_all,3) >= z_coords(layer_WI(1)) & ...
                        G.cells.centroids(cells_WI_all,3) <= z_coords(layer_WI(2)));

% WP
cells_WP_all = findColumnAtPoint(G, xWP, yWP);
cells_WP = cells_WP_all(G.cells.centroids(cells_WP_all,3) >= z_coords(layer_WP(1)) & ...
                        G.cells.centroids(cells_WP_all,3) <= z_coords(layer_WP(2)));

W = [];
W = addWell(W, G, rock_het, cells_WI, ...
    'Type', 'bhp', 'Val', pI, 'Name', 'WI', 'compi', [1 0]);

W = addWell(W, G, rock_het, cells_WP, ...
    'Type', 'bhp', 'Val', pP, 'Name', 'WP', 'compi', [1 0]);
%% ВРЕМЯ
Tmax = 800*day();
dt   = 10*day();
nstep = ceil(Tmax/dt);

schedule = simpleSchedule(repmat(dt, nstep, 1), 'W', W );

%% РАССЧЁТ
result_III = run_calculation(G, rock_het, fluid, schedule, state);

%% СХОДИМОСТЬ
%result_III.QPo(end)
%helpplot(G, W(2), result_III.reports)
%plots(G, nx, ny, nz, Lx, Ly, rock_het, result_III, W);
%dts = [1*day(), 10*day(), 20*day()];
%compare_dt_profiles(G, rock_het, fluid, state, W, Tmax, dts)
%% ---------- 2D HOMOGENIZED MODEL ----------
%% ---------- 2D-СЕТКА ---------------------------------------
% Эффективная пористость (арифметическое среднее)
m_eff = mean(m0 * alpha);
% Эффективная проницаемость (параллельное соединение слоев)
k_eff = mean(k0 * beta);
G2D = cartGrid([nx, ny, 1], [Lx, Ly, 1]);
G2D = computeGeometry(G2D);
%% ---------- ПОРОДА -----------------------------------------
rock2D.poro = m_eff * ones(G2D.cells.num, 1);
rock2D.perm = k_eff * ones(G2D.cells.num, 1);
%% ---------- ФЛЮИД -----------------------------------------
table_data = [
    0.000 0.000 1.000
    0.110 0.054 0.698
    0.316 0.321 0.390
    0.592 0.453 0.180
    0.727 0.586 0.068
    0.764 0.642 0.047
    0.785 0.677 0.038
    0.805 0.707 0.031
    0.827 0.738 0.024
    0.851 0.771 0.018
    0.876 0.807 0.012
    0.903 0.846 0.008
    0.931 0.890 0.004
    0.964 0.942 0.001
    1.000 1.000 0.000
];

mu_w = 1*centi*poise;
mu_o = 5*centi*poise;

rho_w = 1000;
rho_o = 800;

s = table_data(:, 1);
krw = table_data(:, 2);
kro = table_data(:, 3);

fluid2D = initSimpleFluid( ...
    'phases', 'WO', ...
    'mu',  [mu_w, mu_o], ...
    'rho', [1000, 800], ...
    'n',   [a, a]);

%% ---------- НАЧАЛЬНОЕ СОСТОЯНИЕ ----------------------------
p0 = 100*barsa();
state2D = initState(G2D, [], p0, [0 1]);

%% ---------- СКВАЖИНЫ ---------------------------------------
W2D = [];

% Координаты скважин (те же, что в 3D)
xWI = Lx/4;   yWI = Ly/2;
xWP = 3*Lx/4; yWP = Ly/2;

cells_WI_2D = findCell(G2D, [xWI yWI]);
cells_WP_2D = findCell(G2D, [xWP yWP]);

pI = 110*barsa();
pP = 90*barsa();

W2D = addWell(W2D, G2D, rock2D, cells_WI_2D, ...
    'Type', 'bhp', 'Val', pI, ...
    'Name', 'WI', 'compi', [1 0], 'Radius', 0.1, 'Skin', 0.4055);

W2D = addWell(W2D, G2D, rock2D, cells_WP_2D, ...
    'Type', 'bhp', 'Val', pP, ...
    'Name', 'WP', 'compi', [1 0], 'Radius', 0.1, 'Skin', 0.4055);

%% ---------- ВРЕМЯ ------------------------------------------
schedule2D = simpleSchedule(repmat(dt, nstep, 1), 'W', W2D);

%% ---------- РАСЧЁТ -----------------------------------------
result_2D = run_calculation(G2D, rock2D, fluid2D, schedule2D, state2D);
%%
%plot_Q(result_2D.time, result_2D.QI, result_2D.QPo, result_2D.wcP, result_2D.qI, result_2D.qP, result_2D.qPo);
%compare_fields_2D_xy(result_2D.reports, result_III.reports, rock2D, rock_het, nx, ny, nz, x_coords, y_coords, 's', result_2D.iPVs, result_III.iPVs)
fields3d(G2D, result_2D.reports, Lx, Ly, hy, 'p', result_2D.iPVs)
%% findCell
function cellID = findCell(G, point)
    x = point(1);
    y = point(2);

    centroids = G.cells.centroids(:,1:2);

    dist2 = (centroids(:,1) - x).^2 + ...
            (centroids(:,2) - y).^2;

    [~, cellID] = min(dist2);
end