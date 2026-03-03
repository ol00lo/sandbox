clear; close all; clc;
%% ----------------------------
% МОДУЛИ
mrstModule add incomp
mrstModule add ad-core ad-blackoil ad-props mrst-gui

gravity off
%% ----------------------------
% геометрия
nx = 40; ny = 20; nz = 6;
Lx = 400; Ly = 200; Lz = 30;
hx = Lx/nx; hy = Ly/ny; hz = Lz/nz;

G = cartGrid([nx, ny, nz], [Lx, Ly, Lz]);
G = computeGeometry(G);
%% ----------------------------
% однородная порода
m0 = 0.1;           
k0 = 1*darcy();     

rock.poro = m0 * ones(G.cells.num,1);
rock.perm = k0 * ones(G.cells.num,1);
%% ----------------------------
% флюид
Kmu = 0.5;
a = 2;
mu_w = 1*centi*poise;
mu_o = mu_w/Kmu;

fluid = initSimpleFluid( ...
    'phases', 'WO', ...
    'mu',  [mu_w, mu_o], ...
    'rho', [1000, 800], ...
    'n',   [a, a]);
%% ----------------------------
% начальное состояние
p0 = 100*barsa();
state = initState(G, [], p0, [0, 1]);
%% ----------------------------
% идеальные скважины
W = [];
xWI = Lx/4;   yWI = Ly/2;
xWP = 3*Lx/4; yWP = Ly/2;

cells_WI = findColumnAtPoint(G, xWI, yWI);
cells_WP = findColumnAtPoint(G, xWP, yWP);

pI = 110*barsa();
pP = 90*barsa();
%%
W = addWell(W, G, rock, cells_WI, ...
    'Type', 'bhp', 'Val', pI, 'Name', 'WI', 'compi', [1 0]);

W = addWell(W, G, rock, cells_WP, ...
    'Type', 'bhp', 'Val', pP, 'Name', 'WP', 'compi', [1 0]);
%% ----------------------------
% время
Tmax = 800*day();
dt   = 20*day();
nstep = ceil(Tmax/dt);
%%
schedule = simpleSchedule(repmat(dt, nstep, 1), 'W', W);

%% ---------------------------- 
% сценарий I
result_I = run_calculation(G, rock, fluid, schedule, state);
%% ---------------------------- 
% графики
plots(G, nx, ny, nz, Lx, Ly, rock, result_I, W);
%% ---------------------------- 
% неоднородный пласт
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

%% ---------------------------- 
% сценарий II 
result_II = run_calculation(G, rock_het, fluid, schedule, state);

%% ---------------------------- 
% графики
%plots(G, nx, ny, nz, Lx, Ly, rock_het, result_II, W);

%% ---------------------------- 
% сценарий III
zeta_WI = [0, 2/3];   % вскрыты нижние 2/3 пласта
zeta_WP = [1/3, 1];   % верхние 2/3

layer_WI = max(1, round(zeta_WI * nz));
layer_WP = max(1, round(zeta_WP * nz));

%% Скважины
x_coords = unique(G.cells.centroids(:,1));
y_coords = unique(G.cells.centroids(:,2));
z_coords = unique(G.cells.centroids(:,3));

% WI
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

%% РАССЧЁТ
result_III = run_calculation(G, rock_het, fluid, schedule, state);

%% ---------------------------- 
% графики
%plots(G, nx, ny, nz, Lx, Ly, rock_het, result_III, W)

