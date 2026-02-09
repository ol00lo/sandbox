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
plots(G, nx, ny, nz, Lx, Ly, rock_het, result_II, W)

%% ---------------------------- 
% несовершенные скважин
zeta_WI_0 = 0; zeta_WI_1 = 2/3; zeta_WP_0 = 1/3; zeta_WP_1 = 1;

cells_WI_all = findColumnAtPoint(G, xWI, yWI);
cells_WP_all = findColumnAtPoint(G, xWP, yWP);

zWI = G.cells.centroids(cells_WI_all, 3);
zWP = G.cells.centroids(cells_WP_all, 3);

cells_WI = cells_WI_all( zWI >= zeta_WI_0 * Lz & zWI <= zeta_WI_1 * Lz );
cells_WP = cells_WP_all( zWP >= zeta_WP_0 * Lz & zWP <= zeta_WP_1 * Lz );


W= [];
W= addWell(W, G, rock_het, cells_WI, ...
    'Type', 'bhp', 'Val', pI, ...
    'Name', 'WI', 'compi', [1 0]);
W= addWell(W, G, rock_het, cells_WP, ...
    'Type', 'bhp', 'Val', pP, ...
    'Name', 'WP', 'compi', [1 0]);

schedule = simpleSchedule(repmat(dt, nstep, 1), 'W', W );

%% ---------------------------- 
% сценарий III
result_III = run_calculation(G, rock_het, fluid, schedule, state);
%% ---------------------------- 
% графики
plots(G, nx, ny, nz, Lx, Ly, rock_het, result_III, W)

%%


















zeta_WI = [0, 2/3];   % вскрыты нижние 2/3 пласта
zeta_WP = [1/3, 1];   % верхние 2/3

layer_WI = max(1, round(zeta_WI * nz));
layer_WP = max(1, round(zeta_WP * nz));

%% Скважины
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

%% РАССЧЁТ
result_III = run_calculation(G, rock_het, fluid, schedule, state);


%%











%% ---------- 2D HOMOGENIZED MODEL ----------

% Создаём 2D сетку (x-y)
G2D = cartGrid([nx, ny], [Lx, Ly]);
G2D = computeGeometry(G2D);

% Усредняем свойства 3D пласта по высоте
poro_3D = reshape(rock_het.poro, [nx, ny, nz]);
perm_3D  = reshape(rock_het.perm, [nx, ny, nz]);

poro_eff = mean(poro_3D, 3);  % средняя пористость
perm_eff = mean(perm_3D, 3);  % средняя проницаемость

rock2D.poro = poro_eff(:);
rock2D.perm = perm_eff(:);

% Флюид и начальное состояние
fluid2D = fluid;
state2D = initState(G2D, [], p0, [0, 1]);

% Скважины в 2D
cells_WI_2D = findCell2D(G2D, [xWI, yWI]);
cells_WP_2D = findCell2D(G2D, [xWP, yWP]);

W2D = [];
W2D = addWell(W2D, G2D, rock2D, cells_WI_2D, ...
    'Type','bhp','Val',pI,'Name','WI','compi',[1 0]);
W2D = addWell(W2D, G2D, rock2D, cells_WP_2D, ...
    'Type','bhp','Val',pP,'Name','WP','compi',[1 0]);

% Новый schedule для 2D
Tmax = 800*day();
dt   = 20*day();
nstep = ceil(Tmax/dt);
schedule2D = simpleSchedule(repmat(dt, nstep, 1), 'W', W2D);

%% Расчёт 2D задачи
result_2D = run_calculation2D(G2D, rock2D, fluid2D, schedule2D, state2D);

%% Средние по высоте поля 3D для сравнения
% Используем активные ячейки G.cells.num
s3D = zeros(nx, ny, nz);
p3D = zeros(nx, ny, nz);
for k = 1:nz
    layer_cells = (k-1)*nx*ny + (1:nx*ny);
    layer_cells = layer_cells(layer_cells <= G.cells.num);
    s3D(:,:,k) = reshape(reportsIII{end}.s(layer_cells), nx, ny);
    p3D(:,:,k) = reshape(reportsIII{end}.pressure(layer_cells), nx, ny);
end
s_mean3D = mean(s3D, 3);
p_mean3D = mean(p3D, 3);

%% Графики насыщенности
figure;
subplot(1,2,1); imagesc(x_coords, y_coords, s_mean3D'); axis xy; colorbar;
colormap(readmatrix("map_saturation.txt"));
title('3D mean saturation'); xlabel('x [m]'); ylabel('y [m]');

% 2D насыщенность с учётом активных ячеек
s2D = NaN(nx, ny);
for ix = 1:nx
    for iy = 1:ny
        cellId = sub2ind([nx, ny], ix, iy);
        if cellId <= G2D.cells.num
            s2D(ix, iy) = result_2D.reports{end}.s(cellId);
        end
    end
end
subplot(1,2,2); imagesc(x_coords, y_coords, s2D'); axis xy; colorbar;
title('2D homogenized saturation'); xlabel('x [m]'); ylabel('y [m]');

%% Графики давления
figure;
subplot(1,2,1); imagesc(x_coords, y_coords, p_mean3D'/1e5); axis xy; colorbar;
colormap(readmatrix("map_pressure.txt"));
title('3D mean pressure [bar]'); xlabel('x [m]'); ylabel('y [m]');

p2D = NaN(nx, ny);
for ix = 1:nx
    for iy = 1:ny
        cellId = sub2ind([nx, ny], ix, iy);
        if cellId <= G2D.cells.num
            p2D(ix, iy) = result_2D.reports{end}.pressure(cellId);
        end
    end
end
subplot(1,2,2); imagesc(x_coords, y_coords, p2D'/1e5); axis xy; colorbar;
title('2D homogenized pressure [bar]'); xlabel('x [m]'); ylabel('y [m]');

%% Графики скважинных дебитов
figure; hold on;
q_WI_2D = zeros(1,nstep); 
q_WP_2D = zeros(1,nstep);
for it = 1:nstep
    ws = result_2D.reports{it}.wellSol;
    WI_idx = find(strcmp({ws.name}, 'WI'));
    WP_idx = find(strcmp({ws.name}, 'WP'));
    if ~isempty(WI_idx), q_WI_2D(it) = sum([ws(WI_idx).qWs]); end
    if ~isempty(WP_idx), q_WP_2D(it) = sum([ws(WP_idx).qWs]+[ws(WP_idx).qOs]); end
end
plot(result_2D.time, q_WI_2D, 'b', 'LineWidth', 1.5);
plot(result_2D.time, q_WP_2D, 'r', 'LineWidth', 1.5);
xlabel('Time [days]'); ylabel('Well rate'); grid on;
legend('WI 2D','WP 2D'); title('2D well rates');

%% Функция поиска ближайшей ячейки в 2D
function cellIdx = findCell2D(G, point)
    x0 = point(1); y0 = point(2);
    centroids = G.cells.centroids(:,1:2);  % только x и y
    dist2 = sum((centroids - [x0, y0]).^2, 2);
    [~, cellIdx] = min(dist2);
end

%% Универсальная функция расчёта
function result = run_calculation2D(G, rock, fluid, schedule, state)
    model = TwoPhaseOilWaterModel(G, rock, fluid);
    [states, reports] = simulateScheduleAD(state, model, schedule);
    
    time_vec = cumsum(schedule.step.val)/day();
    nstep = numel(states);
    
    qI  = zeros(nstep,1);
    qPo = zeros(nstep,1);
    wcP = zeros(nstep,1);
    
    for it = 1:nstep
        ws = reports{it}.wellSol;
        WI_idx = find(strcmp({ws.name}, 'WI'));
        WP_idx = find(strcmp({ws.name}, 'WP'));
        
        if ~isempty(WI_idx)
            qI(it) = sum([ws(WI_idx).qWs]);
        end
        if ~isempty(WP_idx)
            qPo(it) = sum([ws(WP_idx).qOs]);
            qPw     = sum([ws(WP_idx).qWs]);
            qP      = qPo(it) + qPw;
            wcP(it) = qPw / max(qP, eps);
        end
    end
    
    QI  = cumtrapz(time_vec*day(), qI);
    QPo = cumtrapz(time_vec*day(), qPo);
    
    result.states  = states;
    result.reports = reports;
    result.time    = time_vec;
    result.QI      = QI;
    result.QPo     = QPo;
    result.wcP     = wcP;
end