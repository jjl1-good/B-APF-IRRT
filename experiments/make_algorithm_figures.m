function make_algorithm_figures(sel)
%MAKE_ALGORITHM_FIGURES 生成"全算法路径对比 / 采样密集度 / 消融对比"图件。
% 所有题注与图例均为英文，障碍物（深灰）与各算法路径（彩色）强对比。
%
% 输出到 <repo>/figures：
%   fig2d_paths_<mode>.png      2-D：9 种算法最终路径并排（3x3）
%   fig2d_density_<mode>.png    2-D：9 种算法采样密集度热图（3x3）
%   fig3d_paths_<mode>.png      3-D：9 种算法最终路径并排（3x3）
%   fig3d_density_<mode>.png    3-D：9 种算法采样密集度（x-y 投影，3x3）
%   fig_ablation_2d_len.png     消融：2-D 三场景平均路径长度 + 成功率
%   fig_ablation_2d_first.png   消融：2-D 三场景首解时间
%   fig_ablation_2d_paths.png   消融：2-D 六变体路径叠加
%   fig_ablation_3d.png         消融：3-D（general / ring）长度与首解时间
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
wsRoot = fileparts(rootDir);
outDir = fullfile(rootDir, 'figures');
if ~exist(outDir, 'dir'), mkdir(outDir); end
addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));

if nargin < 1 || isempty(sel)
    sel = {'paths2d', 'density2d', 'overlay2d', 'comp2d', 'paths3d', 'density3d', 'overlay3d', ...
        'comp3d', 'abl2d', 'abl3d', 'summary'};
end
if ischar(sel), sel = {sel}; end

if ismember('paths2d', sel), log_progress('fig: 2D paths'); plot_paths_2d(outDir); end
if ismember('density2d', sel), log_progress('fig: 2D density'); plot_density_2d(outDir); end
if ismember('overlay2d', sel), log_progress('fig: 2D overlay'); plot_overlay_2d(outDir); end
if ismember('comp2d', sel), log_progress('fig: 2D composite'); plot_composite_2d(outDir); end
if ismember('paths3d', sel), log_progress('fig: 3D paths'); plot_paths_3d(outDir); end
if ismember('density3d', sel), log_progress('fig: 3D density'); plot_density_3d(outDir); end
if ismember('overlay3d', sel), log_progress('fig: 3D overlay'); plot_overlay_3d(outDir); end
if ismember('comp3d', sel), log_progress('fig: 3D composite'); plot_composite_3d(outDir); end
if ismember('abl2d', sel), log_progress('fig: 2D ablation'); plot_ablation_2d(outDir); end
if ismember('abl3d', sel), log_progress('fig: 3D ablation'); plot_ablation_3d(outDir); end
if ismember('summary', sel), log_progress('fig: improvement summary'); plot_summary_figure(outDir); end
fprintf('Algorithm comparison figures written to %s\n', outDir);
end

%% ================= 统一样式 =================

function c = alg_color(name)
switch name
    case 'AB-IRRT*',      c = [0.85 0.08 0.08];
    case 'RRT*',          c = [0.25 0.25 0.28];
    case 'Informed-RRT*', c = [0.08 0.35 0.88];
    case 'RRT-Connect',   c = [0.05 0.62 0.28];
    case 'PRM*',          c = [0.55 0.12 0.72];
    case 'FMT*',          c = [0.95 0.55 0.05];
    case 'BIT*',          c = [0.05 0.60 0.72];
    case 'RRT#',          c = [0.88 0.25 0.62];
    case 'APF-only',      c = [0.55 0.55 0.05];
    otherwise,            c = [0 0 0];
end
end

function c = obs_color(), c = [0.30 0.30 0.33]; end
function c = obs_edge(),  c = [0.02 0.02 0.02]; end

function e = eng_scene(s)
%ENG_SCENE 中文场景名 -> 英文图题。
key = {'一般', '狭窄', '广阔', '悬空', '环廊', '悬挑'};
val = {'General', 'Narrow passage', 'Expansive', 'Suspended-obstacle field', ...
    'Ring gallery', 'Overhanging platforms'};
k = find(strcmp(key, s), 1);
if isempty(k), e = s; else, e = val{k}; end
end

function draw_env2d(ax, env)
for k = 1:size(env.squareAll, 1)
    r = env.squareAll(k, :);
    patch(ax, r(1) + [0 r(3) r(3) 0], r(2) + [0 0 r(4) r(4)], obs_color(), ...
        'EdgeColor', obs_edge(), 'LineWidth', 0.8);
end
for k = 1:size(env.round, 1)
    c = env.round(k, :);
    th = linspace(0, 2 * pi, 60);
    patch(ax, c(1) + c(3) * cos(th), c(2) + c(3) * sin(th), obs_color(), ...
        'EdgeColor', obs_edge(), 'LineWidth', 0.8);
end
end

function draw_env3d(ax, env)
for k = 1:numel(env.cube.axisX)
    lo = [env.cube.axisX(k), env.cube.axisY(k), env.cube.axisZ(k)];
    sz = [env.cube.lengthx(k), env.cube.lengthy(k), env.cube.lengthz(k)];
    draw_box3(ax, lo, sz);
end
for k = 1:numel(env.cylinder.X)
    draw_cyl3(ax, [env.cylinder.X(k), env.cylinder.Y(k), env.cylinder.Z(k)], ...
        env.cylinder.radius(k), env.cylinder.lengthZ(k));
end
for k = 1:numel(env.sphere.X)
    draw_sph3(ax, [env.sphere.X(k), env.sphere.Y(k), env.sphere.Z(k)], env.sphere.radius(k));
end
end

function draw_box3(ax, lo, sz)
vx = [0 1 1 0 0 1 1 0] * sz(1) + lo(1);
vy = [0 0 1 1 0 0 1 1] * sz(2) + lo(2);
vz = [0 0 0 0 1 1 1 1] * sz(3) + lo(3);
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];
patch(ax, 'Vertices', [vx(:), vy(:), vz(:)], 'Faces', f, 'FaceColor', obs_color(), ...
    'FaceAlpha', 0.35, 'EdgeColor', obs_edge(), 'LineWidth', 0.7);
end

function draw_cyl3(ax, c, r, h)
[xs, ys, zs] = cylinder(r, 24);
zs = zs * h + c(3);
surf(ax, xs + c(1), ys + c(2), zs, 'FaceColor', obs_color(), 'FaceAlpha', 0.40, ...
    'EdgeColor', 'none');
end

function draw_sph3(ax, c, r)
[xs, ys, zs] = sphere(18);
surf(ax, r * xs + c(1), r * ys + c(2), r * zs + c(3), 'FaceColor', obs_color(), ...
    'FaceAlpha', 0.40, 'EdgeColor', 'none');
end

function draw_markers(ax, env, dim)
if dim == 2
    plot(ax, env.start(1), env.start(2), 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0 0 0], ...
        'MarkerEdgeColor', [1 1 1], 'LineWidth', 1.0);
    plot(ax, env.goal(1), env.goal(2), 'p', 'MarkerSize', 15, 'MarkerFaceColor', [0.05 0.75 0.20], ...
        'MarkerEdgeColor', [0 0 0], 'LineWidth', 1.0);
else
    plot3(ax, env.start(1), env.start(2), env.start(3), 'o', 'MarkerSize', 7, ...
        'MarkerFaceColor', [0 0 0], 'MarkerEdgeColor', [1 1 1], 'LineWidth', 0.8);
    plot3(ax, env.goal(1), env.goal(2), env.goal(3), 'p', 'MarkerSize', 13, ...
        'MarkerFaceColor', [0.05 0.75 0.20], 'MarkerEdgeColor', [0 0 0], 'LineWidth', 0.8);
end
end

function save_fig(fig, outDir, name, doVector)
% 统一导出：print + 显式纸面尺寸，保证导出图与屏幕版式等比（面板不被拉伸）。
% 注：exportgraphics 对这类大尺寸无界面 figure 会重排版式，把方形面板导成 1:3.4
%     （已由 check_panel_geometry.m 量化），因此改用 print。
% doVector=false 时只导 PNG：三维场景/热图类图件的矢量 PDF 体积大而且
%     奇慢（单张可达数分钟），且其内容是半透明面片与海量点，矢量并无优势。
if nargin < 4, doVector = true; end
w = fig.Position(3); h = fig.Position(4);
set(fig, 'PaperUnits', 'inches', 'PaperPositionMode', 'manual', ...
    'PaperPosition', [0 0 w / 96, h / 96]);
print(fig, fullfile(outDir, [name, '.png']), '-dpng', '-r200');
if doVector
    try
        print(fig, fullfile(outDir, [name, '.pdf']), '-dpdf');
    catch
    end
end
end

%% ================= 单算法运行（固定种子，供图件使用） =================

function R = run_alg_2d(alg, env, mode, ci)
% 使用与主实验第 1 次运行相同的随机种子（seed = 20260903 + 1000*c + 1）。
% 会话内缓存：同一场景、同一算法只运行一次，便于多个图件复用同一次运行。
persistent C2D
if isempty(C2D), C2D = containers.Map('KeyType', 'char', 'ValueType', 'any'); end
key = sprintf('%s|%s', mode, alg);
if isKey(C2D, key), R = C2D(key); return; end
cf = fig_cache_path('2d', mode, alg, 2000);
if exist(cf, 'file'), S = load(cf); R = S.R; C2D(key) = R; return; end
rng(seed_for_figure_run('2d', mode), 'twister');   % 配对代表性运行
o = struct('maxIter', 2000, 'step', 4, 'goalBias', 0.08, 'rewireRadius', 14);
isFree = @(a, b) collisionChecking(a, b, env.squareAll, env.round);
R = struct('path', zeros(0, 2), 'samples', zeros(0, 2), 'ok', false, 'L', NaN, 'seed', zeros(0, 2));
switch alg
    case 'RRT*'
        [p, i] = rrtstar2d(env, o); R.ok = i.success; R.samples = i.samples;
    case 'Informed-RRT*'
        [p, i] = informed_rrtstar2d(env, [], o); R.ok = i.success; R.samples = i.samples;
    case 'RRT-Connect'
        [p, i] = rrtconnect2d(env, o); R.ok = i.success;
        if isfield(i, 'samples'), R.samples = i.samples; end
    case 'PRM*'
        po = o; po.maxNodes = o.maxIter; po.gamma = 1.0; po.kMax = 12;
        [p, i] = prmstar_nd(env.bounds, env.start, env.goal, isFree, po); R.ok = i.success; R.samples = i.samples;
    case 'FMT*'
        po = o; po.maxNodes = o.maxIter; po.gamma = 0.8;
        [p, i] = fmtstar_nd(env.bounds, env.start, env.goal, isFree, po); R.ok = i.success; R.samples = i.samples;
    case 'BIT*'
        po = o; po.maxNodes = o.maxIter; po.gamma = 0.9;
        [p, i] = bitstar_nd(env.bounds, env.start, env.goal, isFree, po); R.ok = i.success; R.samples = i.samples;
    case 'RRT#'
        [p, i] = rrtsharp_nd(env.bounds, env.start, env.goal, isFree, o); R.ok = i.success; R.samples = i.samples;
    case 'APF-only'
        [p, i] = stable_apf2d(env, struct('useFallback', false, 'step', o.step)); R.ok = i.success;
    case 'AB-IRRT*'
        bo = o; bo.pbias = 0.5; bo.dynamicRewire = true; bo.rewireMax = 16; bo.useFallback = true;
        [~, ~, d] = bair_core2d(mode, bo); p = d.path; R.ok = d.success;
        R.samples = d.rrt.samples;
        if isfield(d, 'apfPath'), R.seed = d.apfPath; end
    otherwise
        error('unknown algorithm %s', alg);
end
R.path = p;
if size(p, 1) >= 2, R.L = sum(sqrt(sum(diff(p).^2, 2))); end
C2D(key) = R;
save(cf, 'R');
end

function R = run_alg_3d(alg, env, mode)
% 一、与主实验一致的预算 6000 与种子（各场景 c = 1）。
persistent C3D
if isempty(C3D), C3D = containers.Map('KeyType', 'char', 'ValueType', 'any'); end
key = sprintf('%s|%s', mode, alg);
if isKey(C3D, key), R = C3D(key); return; end
cf = fig_cache_path('3d', mode, alg, 6000);
if exist(cf, 'file'), S = load(cf); R = S.R; C3D(key) = R; return; end
rng(seed_for_figure_run('3d', mode), 'twister');   % 配对代表性运行
o = struct('maxIter', 6000, 'step', 6, 'goalBias', 0.08, 'rewireRadius', 18);
isFree = @(a, b) collisionChecking3D(a, b, env);
R = struct('path', zeros(0, 3), 'samples', zeros(0, 3), 'ok', false, 'L', NaN, 'seed', zeros(0, 3));
switch alg
    case 'RRT*'
        [p, i] = rrtstar3d(env, o); R.ok = i.success; R.samples = i.samples;
    case 'Informed-RRT*'
        [p, i] = informed_rrtstar3d(env, [], o); R.ok = i.success; R.samples = i.samples;
    case 'RRT-Connect'
        [p, i] = rrtconnect3d(env, o); R.ok = i.success;
        if isfield(i, 'samples'), R.samples = i.samples; end
    case 'PRM*'
        po = o; po.maxNodes = o.maxIter; po.gamma = 1.0; po.kMax = 12;
        [p, i] = prmstar_nd(env.bounds, env.start, env.goal, isFree, po); R.ok = i.success; R.samples = i.samples;
    case 'FMT*'
        po = o; po.maxNodes = o.maxIter; po.gamma = 0.7;
        [p, i] = fmtstar_nd(env.bounds, env.start, env.goal, isFree, po); R.ok = i.success; R.samples = i.samples;
    case 'BIT*'
        po = o; po.maxNodes = o.maxIter; po.gamma = 0.65;
        [p, i] = bitstar_nd(env.bounds, env.start, env.goal, isFree, po); R.ok = i.success; R.samples = i.samples;
    case 'RRT#'
        [p, i] = rrtsharp_nd(env.bounds, env.start, env.goal, isFree, o); R.ok = i.success; R.samples = i.samples;
    case 'APF-only'
        [p, i] = stable_apf3d(env, struct('useFallback', false, 'step', o.step)); R.ok = i.success;
    case 'AB-IRRT*'
        bo = o; bo.pbias = 0.1; bo.dynamicRewire = true; bo.rewireMax = 30; bo.useFallback = true;
        [~, ~, d] = bair_core3d(mode, bo); p = d.path; R.ok = d.success;
        R.samples = d.rrt.samples;
        if isfield(d, 'apfPath'), R.seed = d.apfPath; end
    otherwise
        error('unknown algorithm %s', alg);
end
R.path = p;
if size(p, 1) >= 2, R.L = sum(sqrt(sum(diff(p).^2, 2))); end
C3D(key) = R;
save(cf, 'R');
end

%% ================= 2-D 全算法路径 =================

function plot_paths_2d(outDir)
modes = {'y', 'n', 'g'};
names = {'General 2-D environment (150 x 150)', 'Narrow-passage 2-D environment', ...
    'Expansive 2-D environment (300 x 300)'};
tags = {'y', 'n', 'g'};
algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
for si = 1:numel(modes)
    env = bair_env2d(modes{si});
    fig = figure('Visible', 'off', 'Position', [30 30 1700 1000]);
    for k = 1:numel(algs)
        ax = subplot(2, 4, k); hold(ax, 'on'); axis(ax, 'equal'); box(ax, 'on');
        draw_env2d(ax, env);
        R = run_alg_2d(algs{k}, env, modes{si}, si);
        if ~isempty(R.seed) && size(R.seed, 1) >= 2
            plot(ax, R.seed(:, 1), R.seed(:, 2), '--', 'Color', [0.92 0.55 0.55], 'LineWidth', 1.3);
        end
        if size(R.path, 1) >= 2
            plot(ax, R.path(:, 1), R.path(:, 2), '-', 'Color', alg_color(algs{k}), 'LineWidth', 2.6);
            plot(ax, R.path(:, 1), R.path(:, 2), '.', 'Color', alg_color(algs{k}), 'MarkerSize', 6);
        end
        draw_markers(ax, env, 2);
        axis(ax, env.bounds(1:4)); pbaspect(ax, [1 1 1]);
        if R.ok
            ttl = len_title(algs{k}, R, mean_len_lookup('2d', modes{si}, algs{k}), summary_rank('2d', modes{si}, algs{k}));
        else
            ttl = sprintf('%s  |  no solution', algs{k});
        end
        title(ax, ttl, 'FontSize', 11.5, 'FontWeight', 'bold', 'Color', alg_color(algs{k}));
        set(ax, 'FontSize', 9);
    end
    sgtitle(fig, sprintf('Final paths of all eight algorithms (identical seed and budget): %s', names{si}), ...
        'FontSize', 14, 'FontWeight', 'bold');
    save_fig(fig, outDir, sprintf('fig2d_paths_%s', tags{si}));
    close(fig);
end
end

%% ================= 2-D 采样密集度 =================

function plot_density_2d(outDir)
modes = {'y', 'n', 'g'};
names = {'General 2-D environment (150 x 150)', 'Narrow-passage 2-D environment', ...
    'Expansive 2-D environment (300 x 300)'};
tags = {'y', 'n', 'g'};
algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
bin = 4;
for si = 1:numel(modes)
    env = bair_env2d(modes{si});
    nA = numel(algs);
    H = cell(1, nA); R = cell(1, nA);
    for k = 1:nA
        R{k} = run_alg_2d(algs{k}, env, modes{si}, si);
        H{k} = density_hist(env, R{k}.samples, bin);
    end
    fig = figure('Visible', 'off', 'Position', [30 30 1700 1000]);
    for k = 1:nA
        ax = subplot(2, 4, k); hold(ax, 'on'); box(ax, 'on');
        R = run_alg_2d(algs{k}, env, modes{si}, si);
        density_panel(ax, env, R.samples, R.path, bin);
        axis(ax, env.bounds(1:4)); axis(ax, 'equal'); pbaspect(ax, [1 1 1]);
        title(ax, density_title(algs{k}, R, mean_len_lookup('2d', modes{si}, algs{k}), summary_rank('2d', modes{si}, algs{k})), 'FontSize', 11.5, 'FontWeight', 'bold', ...
            'Color', alg_color(algs{k}));
        set(ax, 'FontSize', 9);
    end
    add_density_colorbar(fig, bin);
    sgtitle(fig, sprintf('Sampling density of all eight algorithms (one run, cell = %d x %d): %s', ...
        bin, bin, names{si}), 'FontSize', 14, 'FontWeight', 'bold');
    save_fig(fig, outDir, sprintf('fig2d_density_%s', tags{si}), false);
    close(fig);
end
end

function h = density_hist(env, S, bin)
xe = env.bounds(1):bin:env.bounds(2);
ye = env.bounds(3):bin:env.bounds(4);
if isempty(S)
    h = zeros(numel(ye) - 1, numel(xe) - 1);
else
    h = histcounts2(S(:, 1), S(:, 2), xe, ye)';
end
end

function t = density_title(alg, R, m, rk)
n = size(R.samples, 1);
if n > 0
    if nargin > 3 && isfinite(m) && isfinite(rk)
        t = sprintf('%s  |  %d samples  |  mean L = %.1f (rank %d/8)', alg, n, m, rk);
    elseif nargin > 2 && isfinite(m)
        t = sprintf('%s  |  %d samples (mean L = %.1f)', alg, n, m);
    else
        t = sprintf('%s  |  %d samples', alg, n);
    end
elseif size(R.path, 1) >= 2
    if nargin > 3 && isfinite(m) && isfinite(rk)
        t = sprintf('%s  |  no sample set (path shown)  |  mean L = %.1f (rank %d/8)', alg, m, rk);
    elseif nargin > 2 && isfinite(m)
        t = sprintf('%s  |  no sample set (path shown, mean L = %.1f)', alg, m);
    else
        t = sprintf('%s  |  no sample set (path shown)', alg);
    end
else
    t = sprintf('%s  |  no solution', alg);
end
end

function t = len_title(alg, R, m, rk)
if R.ok && isfinite(R.L)
    if nargin > 3 && isfinite(m) && isfinite(rk)
        t = sprintf('%s  |  mean L = %.1f (rank %d/8)  |  this run %.1f', alg, m, rk, R.L);
    elseif nargin > 2 && isfinite(m)
        t = sprintf('%s  |  mean L = %.1f  |  this run %.1f', alg, m, R.L);
    else
        t = sprintf('%s  |  L = %.1f', alg, R.L);
    end
else
    t = sprintf('%s  |  no solution', alg);
end
end

function r = summary_rank(dim, scene, alg)
%SUMMARY_RANK 按 50 次平均路径长度排名（1 = 最短）。
algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
v = summary_lookup(dim, scene, alg, 'meanLength');
c = 0;
for j = 1:numel(algs)
    w = summary_lookup(dim, scene, algs{j}, 'meanLength');
    if isfinite(v) && isfinite(w) && w < v - 1e-9
        c = c + 1;
    end
end
r = c + 1;
end

function m = mean_len_lookup(dim, scene, alg)
%MEAN_LEN_LOOKUP 从 summary_*.csv 读取 50 次运行的平均路径长度（面板标题标注用）。
m = summary_lookup(dim, scene, alg, 'meanLength');
end

function v = summary_lookup(dim, scene, alg, field)
%SUMMARY_LOOKUP 读取 summary_*.csv 中指定字段（meanLength / successRate / meanTime / ...）。
persistent CACHE
if isempty(CACHE), CACHE = containers.Map('KeyType', 'char', 'ValueType', 'any'); end
entryDir = fileparts(mfilename('fullpath'));
key = sprintf('%s|%s', dim, scene);
if ~isKey(CACHE, key)
    M = containers.Map('KeyType', 'char', 'ValueType', 'any');
    if strcmp(dim, '2d')
        cn = struct('y', '一般', 'n', '狭窄', 'g', '广阔');
        f = fullfile(entryDir, 'summary_2d.csv');
        if exist(f, 'file')
            T = readtable(f, 'Encoding', 'UTF-8', 'VariableNamingRule', 'preserve');
            for i = 1:height(T)
                if strcmp(char(string(T.scenario(i))), cn.(scene))
                    M(char(string(T.algorithm(i)))) = entry_struct(T, i);
                end
            end
        end
    else
        % 重设计后的三维一般场景使用模式名 general_v2，但结果文件仍为 summary_3d_general.csv
        fname = scene;
        if strcmp(scene, 'general_v2'), fname = 'general'; end
        f = fullfile(entryDir, sprintf('summary_3d_%s.csv', fname));
        if exist(f, 'file')
            T = readtable(f, 'Encoding', 'UTF-8', 'VariableNamingRule', 'preserve');
            for i = 1:height(T)
                M(char(string(T.algorithm(i)))) = entry_struct(T, i);
            end
        end
    end
    CACHE(key) = M;
end
M = CACHE(key);
if isKey(M, alg) && isfield(M(alg), field)
    v = M(alg).(field);
else
    v = NaN;
end
end

function e = entry_struct(T, i)
e = struct('meanLength', double(T.meanLength(i)), 'stdLength', double(T.stdLength(i)), ...
    'successRate', double(T.successRate(i)), 'meanTime', double(T.meanTime(i)), ...
    'meanFirstSol', double(T.meanFirstSol(i)), 'meanMinClear', double(T.meanMinClear(i)));
end

function f = fig_cache_path(dim, scene, name, budget)
%FIG_CACHE_PATH 图件运行结果的磁盘缓存路径（避免每次重出图都重跑算法）。
entryDir = fileparts(mfilename('fullpath'));
d = fullfile(entryDir, '_figcache');
if ~exist(d, 'dir'), mkdir(d); end
nm = regexprep(name, '[^A-Za-z0-9]', '');
f = fullfile(d, sprintf('%s_%s_%s_%d.mat', dim, scene, nm, budget));
end

function s = seed_for_figure_run(dim, scene)
%SEED_FOR_FIGURE_RUN 选“代表性运行”的种子。
%   在 50 次配对运行中，选出使“各算法长度与其自身中位数的相对偏差之和”最小的那一次，
%   这样八种算法共用同一种子（保持配对对比），且每个算法展示的是典型表现而非偶然抽到的一次。
persistent CACHE
if isempty(CACHE), CACHE = containers.Map('KeyType', 'char', 'ValueType', 'any'); end
key = sprintf('%s|%s', dim, scene);
if isKey(CACHE, key), s = CACHE(key); return; end
entryDir = fileparts(mfilename('fullpath'));
cn = struct('y', '一般', 'n', '狭窄', 'g', '广阔');
s = 20261904;
if strcmp(dim, '2d')
    f = fullfile(entryDir, 'results_2d_v2.csv'); nameField = 'algorithm'; scen = cn.(scene);
elseif strcmp(dim, '3d')
    f = fullfile(entryDir, sprintf('results_3d_v2_%s.csv', scene)); nameField = 'algorithm'; scen = '';
else
    f = fullfile(entryDir, 'ablation_2d.csv'); nameField = 'variant'; scen = cn.(scene);
end
if exist(f, 'file')
    T = readtable(f, 'Encoding', 'UTF-8', 'VariableNamingRule', 'preserve');
    if ~isempty(scen)
        T = T(strcmp(string(T.scenario), scen), :);
    end
    names = unique(string(T.(nameField)), 'stable');
    runs = unique(double(T.run));
    cost = inf(size(runs));
    for ri = 1:numel(runs)
        c = 0; cnt = 0;
        for j = 1:numel(names)
            m = strcmp(string(T.(nameField)), names(j));
            L = double(T.length(m)); ok = logical(T.success(m)); rn = double(T.run(m));
            Lg = L(ok & isfinite(L));
            if isempty(Lg), continue; end
            med = median(Lg);
            Lr = L(rn == runs(ri) & ok & isfinite(L));
            if isempty(Lr)
                c = c + 5;
            else
                c = c + abs(Lr(1) - med) / med;
            end
            cnt = cnt + 1;
        end
        if cnt > 0, cost(ri) = c / cnt; end
    end
    [~, best] = min(cost);
    rr = runs(best);
    m = double(T.run) == rr;
    if any(m)
        s = double(T.seed(find(m, 1)));
        fprintf('figure representative run: %s | %s -> run %d (seed %d), mean relative deviation %.4f\n', ...
            dim, scene, rr, s, cost(best));
    end
end
CACHE(key) = s;
end

function s = seed_for_median_run(dim, scene, alg)
%SEED_FOR_MEDIAN_RUN 取“最终长度最接近中位数”的那次运行的种子作为代表运行。
%   这样面板展示的是该算法的典型表现，而不是偶然抽到的一次运气。
persistent CACHE
if isempty(CACHE), CACHE = containers.Map('KeyType', 'char', 'ValueType', 'any'); end
key = sprintf('%s|%s|%s', dim, scene, alg);
if isKey(CACHE, key), s = CACHE(key); return; end
entryDir = fileparts(mfilename('fullpath'));
cn = struct('y', '一般', 'n', '狭窄', 'g', '广阔');
s = 20261904;
if strcmp(dim, '2d')
    f = fullfile(entryDir, 'results_2d_v2.csv'); nameField = 'algorithm'; scen = cn.(scene);
elseif strcmp(dim, '3d')
    f = fullfile(entryDir, sprintf('results_3d_v2_%s.csv', scene)); nameField = 'algorithm'; scen = '';
else
    f = fullfile(entryDir, 'ablation_2d.csv'); nameField = 'variant'; scen = cn.(scene);
end
if exist(f, 'file')
    T = readtable(f, 'Encoding', 'UTF-8', 'VariableNamingRule', 'preserve');
    sel = strcmp(string(T.(nameField)), alg);
    if ~isempty(scen)
        sel = sel & strcmp(string(T.scenario), scen);
    end
    Lall = double(T.length(sel)); Sall = double(T.seed(sel)); okall = logical(T.success(sel));
    keep = okall & isfinite(Lall);
    L = Lall(keep); S = Sall(keep);
    if ~isempty(L)
        med = median(L);
        [~, j] = min(abs(L - med));
        s = S(j);
    end
end
CACHE(key) = s;
end

function add_density_colorbar(fig, bin, pos)
if nargin < 3 || isempty(pos), pos = [0.943 0.06 0.013 0.855]; end
cbax = axes(fig, 'Position', pos, 'Visible', 'off');
colormap(cbax, turbo(256));
clim(cbax, [0 1]);
cb = colorbar(cbax);
set(cb, 'Ticks', 0:0.25:1, 'TickLabels', {'0', '0.25', '0.50', '0.75', '1'});
cb.Label.String = sprintf('relative sampling density\n(per panel; %d x %d cell)', bin, bin);
cb.Label.FontSize = 9;
end

function density_panel(ax, env, S, path, bin)
xe = env.bounds(1):bin:env.bounds(2);
ye = env.bounds(3):bin:env.bounds(4);
xc = xe(1:end - 1) + bin / 2;
yc = ye(1:end - 1) + bin / 2;
H = density_hist(env, S, bin);
hmax = max(H(:)); if hmax <= 0, hmax = 1; end
h = imagesc(ax, xc, yc, H);
set(ax, 'YDir', 'normal', 'Color', 'w');
colormap(ax, turbo(256));
clim(ax, [0 hmax]);
A = zeros(size(H));
nz = H > 0;
A(nz) = 0.20 + 0.80 * sqrt(H(nz) / hmax);
set(h, 'AlphaData', A);
hold(ax, 'on');
% 真实采样点（与热图同坐标，便于核对）
if ~isempty(S)
    scatter(ax, S(:, 1), S(:, 2), 3, [0.05 0.05 0.05], 'filled', 'MarkerFaceAlpha', 0.30);
end
% 障碍物半透明覆盖（保留清晰轮廓，与彩色热图强对比）
for k = 1:size(env.squareAll, 1)
    r = env.squareAll(k, :);
    patch(ax, r(1) + [0 r(3) r(3) 0], r(2) + [0 0 r(4) r(4)], [0.15 0.15 0.17], ...
        'FaceAlpha', 0.75, 'EdgeColor', [1 1 1], 'LineWidth', 0.8);
end
for k = 1:size(env.round, 1)
    c = env.round(k, :);
    th = linspace(0, 2 * pi, 60);
    patch(ax, c(1) + c(3) * cos(th), c(2) + c(3) * sin(th), [0.15 0.15 0.17], ...
        'FaceAlpha', 0.75, 'EdgeColor', [1 1 1], 'LineWidth', 0.8);
end
if size(path, 1) >= 2
    plot(ax, path(:, 1), path(:, 2), '-', 'Color', [1 1 1], 'LineWidth', 3.0);
    plot(ax, path(:, 1), path(:, 2), '-', 'Color', [0.10 0.10 0.10], 'LineWidth', 1.4);
end
draw_markers(ax, env, 2);
end

%% ================= 九算法叠加对比图（二维） =================

function plot_overlay_2d(outDir)
modes = {'y', 'n', 'g'};
names = {'General 2-D environment (150 x 150)', 'Narrow-passage 2-D environment', ...
    'Expansive 2-D environment (300 x 300)'};
algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
hAll = gobjects(1, numel(algs));
axLast = [];
fig = figure('Visible', 'off', 'Position', [30 30 1800 640]);
for si = 1:numel(modes)
    env = bair_env2d(modes{si});
    ax = subplot(1, 3, si); hold(ax, 'on'); box(ax, 'on');
    draw_env2d(ax, env);
    for k = 1:numel(algs)
        R = run_alg_2d(algs{k}, env, modes{si}, si);
        if size(R.path, 1) >= 2
            h = plot(ax, R.path(:, 1), R.path(:, 2), '-', 'Color', alg_color(algs{k}), 'LineWidth', 2.4);
            if ~isgraphics(hAll(k)), hAll(k) = h; end
        end
    end
    draw_markers(ax, env, 2);
    axis(ax, env.bounds(1:4)); axis(ax, 'equal'); pbaspect(ax, [1 1 1]);
    title(ax, names{si}, 'FontSize', 12, 'FontWeight', 'bold');
    set(ax, 'FontSize', 9);
    axLast = ax;
end
keep = isgraphics(hAll);
lgd = legend(axLast, hAll(keep), algs(keep), 'Orientation', 'horizontal', 'FontSize', 9, 'Box', 'off');
set(lgd, 'Position', [0.10 0.012 0.80 0.05]);
sgtitle(fig, 'Final paths of all eight algorithms in the same axes (identical seed and budget)', ...
    'FontSize', 14, 'FontWeight', 'bold');
save_fig(fig, outDir, 'fig2d_overlay_all');
close(fig);
end

%% ================= 2-D 集成图（路径 + 采样密集度） =================

function plot_composite_2d(outDir)
modes = {'y', 'n', 'g'};
names = {'General 2-D environment (150 x 150)', 'Narrow-passage 2-D environment', ...
    'Expansive 2-D environment (300 x 300)'};
tags = {'y', 'n', 'g'};
algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
bin = 4;
for si = 1:numel(modes)
    env = bair_env2d(modes{si});
    nA = numel(algs);
    fig = figure('Visible', 'off', 'Position', [20 20 1700 1800]);
    for k = 1:nA
        ax = subplot(4, 4, k); hold(ax, 'on'); axis(ax, 'equal'); box(ax, 'on');
        draw_env2d(ax, env);
        R = run_alg_2d(algs{k}, env, modes{si}, si);
        if ~isempty(R.seed) && size(R.seed, 1) >= 2
            plot(ax, R.seed(:, 1), R.seed(:, 2), '--', 'Color', [0.92 0.55 0.55], 'LineWidth', 1.2);
        end
        if size(R.path, 1) >= 2
            plot(ax, R.path(:, 1), R.path(:, 2), '-', 'Color', alg_color(algs{k}), 'LineWidth', 2.6);
            plot(ax, R.path(:, 1), R.path(:, 2), '.', 'Color', alg_color(algs{k}), 'MarkerSize', 6);
        end
        draw_markers(ax, env, 2);
        axis(ax, env.bounds(1:4)); pbaspect(ax, [1 1 1]);
        if R.ok
            ttl = len_title(algs{k}, R, mean_len_lookup('2d', modes{si}, algs{k}), summary_rank('2d', modes{si}, algs{k}));
        else
            ttl = sprintf('%s  |  no solution', algs{k});
        end
        title(ax, ttl, 'FontSize', 12, 'FontWeight', 'bold', 'Color', alg_color(algs{k}));
        set(ax, 'FontSize', 9);
    end
    for k = 1:nA
        ax = subplot(4, 4, nA + k); hold(ax, 'on'); box(ax, 'on');
        R = run_alg_2d(algs{k}, env, modes{si}, si);
        density_panel(ax, env, R.samples, R.path, bin);
        axis(ax, env.bounds(1:4)); axis(ax, 'equal'); pbaspect(ax, [1 1 1]);
        title(ax, density_title(algs{k}, R, mean_len_lookup('2d', modes{si}, algs{k}), summary_rank('2d', modes{si}, algs{k})), 'FontSize', 12, 'FontWeight', 'bold', ...
            'Color', alg_color(algs{k}));
        set(ax, 'FontSize', 9);
    end
    add_density_colorbar(fig, bin, [0.947 0.055 0.011 0.43]);
    annotation(fig, 'textbox', [0.004 0.815 0.05 0.03], 'String', '(a) final path', ...
        'EdgeColor', 'none', 'FontSize', 13, 'FontWeight', 'bold');
    annotation(fig, 'textbox', [0.004 0.315 0.05 0.03], 'String', '(b) sampling density', ...
        'EdgeColor', 'none', 'FontSize', 13, 'FontWeight', 'bold');
    sgtitle(fig, sprintf('Eight algorithms under identical seed and budget: %s', names{si}), ...
        'FontSize', 15, 'FontWeight', 'bold');
    save_fig(fig, outDir, sprintf('fig2d_all_%s', tags{si}), false);
    close(fig);
end
end

%% ================= 3-D 全算法路径 =================

function plot_paths_3d(outDir)
modes = {'general_v2', 'narrow', 'suspended', 'ring', 'overhang'};
names = {'General 3-D', 'Narrow 3-D', 'Suspended-obstacle field', ...
    'Cylindrical ring gallery', 'Overhanging platforms'};
algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
for si = 1:numel(modes)
    env = bair_env3d(modes{si});
    fig = figure('Visible', 'off', 'Position', [30 30 1700 1000]);
    for k = 1:numel(algs)
        ax = subplot(2, 4, k); hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
        draw_env3d(ax, env);
        R = run_alg_3d(algs{k}, env, modes{si});
        if ~isempty(R.seed) && size(R.seed, 1) >= 2
            plot3(ax, R.seed(:, 1), R.seed(:, 2), R.seed(:, 3), '--', 'Color', [0.92 0.55 0.55], 'LineWidth', 1.1);
        end
        if size(R.path, 1) >= 2
            plot3(ax, R.path(:, 1), R.path(:, 2), R.path(:, 3), '-', 'Color', alg_color(algs{k}), 'LineWidth', 2.8);
            plot3(ax, R.path(:, 1), R.path(:, 2), R.path(:, 3), '.', 'Color', alg_color(algs{k}), 'MarkerSize', 7);
        end
        draw_markers(ax, env, 3);
        axis(ax, [env.bounds(1:2), env.bounds(3:4), env.bounds(5:6)]);
        daspect(ax, [1 1 1]); pbaspect(ax, [1 1 1]);
        set(ax, 'SortMethod', 'childorder');
        view(ax, -37.5, 22);
        if R.ok
            ttl = len_title(algs{k}, R, mean_len_lookup('3d', modes{si}, algs{k}), summary_rank('3d', modes{si}, algs{k}));
        else
            ttl = sprintf('%s  |  no solution', algs{k});
        end
        title(ax, ttl, 'FontSize', 11, 'FontWeight', 'bold', 'Color', alg_color(algs{k}));
        set(ax, 'FontSize', 8);
    end
    sgtitle(fig, sprintf('Final paths of all eight algorithms (identical seed and budget): %s', names{si}), ...
        'FontSize', 14, 'FontWeight', 'bold');
    save_fig(fig, outDir, sprintf('fig3d_paths_%s', modes{si}), false);
    close(fig);
end
end

%% ================= 3-D 采样密集度（x-y 投影） =================

function plot_density_3d(outDir)
modes = {'general_v2', 'narrow', 'suspended', 'ring', 'overhang'};
names = {'General 3-D', 'Narrow 3-D', 'Suspended-obstacle field', ...
    'Cylindrical ring gallery', 'Overhanging platforms'};
algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
bin = 6;
for si = 1:numel(modes)
    env = bair_env3d(modes{si});
    nA = numel(algs);
    H = cell(1, nA); R = cell(1, nA);
    for k = 1:nA
        R{k} = run_alg_3d(algs{k}, env, modes{si});
        H{k} = density_hist(env, R{k}.samples, bin);
    end
    fig = figure('Visible', 'off', 'Position', [30 30 1700 1000]);
    for k = 1:nA
        ax = subplot(2, 4, k); hold(ax, 'on'); box(ax, 'on');
        R = run_alg_3d(algs{k}, env, modes{si});
        density_panel_3d(ax, env, R.samples, R.path, bin);
        axis(ax, env.bounds(1:4)); axis(ax, 'equal'); pbaspect(ax, [1 1 1]);
        title(ax, density_title(algs{k}, R, mean_len_lookup('3d', modes{si}, algs{k}), summary_rank('3d', modes{si}, algs{k})), 'FontSize', 11, 'FontWeight', 'bold', ...
            'Color', alg_color(algs{k}));
        set(ax, 'FontSize', 8);
    end
    add_density_colorbar(fig, bin);
    sgtitle(fig, sprintf('Sampling density (x-y projection, one run, cell = %d x %d): %s', ...
        bin, bin, names{si}), 'FontSize', 14, 'FontWeight', 'bold');
    save_fig(fig, outDir, sprintf('fig3d_density_%s', modes{si}), false);
    close(fig);
end
end

function density_panel_3d(ax, env, S, path, bin)
xe = env.bounds(1):bin:env.bounds(2);
ye = env.bounds(3):bin:env.bounds(4);
xc = xe(1:end - 1) + bin / 2;
yc = ye(1:end - 1) + bin / 2;
H = density_hist(env, S, bin);
hmax = max(H(:)); if hmax <= 0, hmax = 1; end
h = imagesc(ax, xc, yc, H);
set(ax, 'YDir', 'normal', 'Color', 'w');
colormap(ax, turbo(256));
clim(ax, [0 hmax]);
A = zeros(size(H));
nz = H > 0;
A(nz) = 0.20 + 0.80 * sqrt(H(nz) / hmax);
set(h, 'AlphaData', A);
hold(ax, 'on');
% 真实采样点（x-y 投影）
if ~isempty(S)
    scatter(ax, S(:, 1), S(:, 2), 3, [0.05 0.05 0.05], 'filled', 'MarkerFaceAlpha', 0.30);
end
% 障碍物在 x-y 平面上的投影轮廓
for k = 1:numel(env.cube.axisX)
    r = [env.cube.axisX(k), env.cube.axisY(k), env.cube.lengthx(k), env.cube.lengthy(k)];
    patch(ax, r(1) + [0 r(3) r(3) 0], r(2) + [0 0 r(4) r(4)], [0.15 0.15 0.17], ...
        'FaceAlpha', 0.75, 'EdgeColor', [1 1 1], 'LineWidth', 0.8);
end
for k = 1:numel(env.cylinder.X)
    c = [env.cylinder.X(k), env.cylinder.Y(k), env.cylinder.radius(k)];
    th = linspace(0, 2 * pi, 60);
    patch(ax, c(1) + c(3) * cos(th), c(2) + c(3) * sin(th), [0.15 0.15 0.17], ...
        'FaceAlpha', 0.75, 'EdgeColor', [1 1 1], 'LineWidth', 0.8);
end
for k = 1:numel(env.sphere.X)
    c = [env.sphere.X(k), env.sphere.Y(k), env.sphere.radius(k)];
    th = linspace(0, 2 * pi, 60);
    patch(ax, c(1) + c(3) * cos(th), c(2) + c(3) * sin(th), [0.15 0.15 0.17], ...
        'FaceAlpha', 0.45, 'EdgeColor', [1 1 1], 'LineWidth', 0.8, 'LineStyle', '--');
end
if size(path, 1) >= 2
    plot(ax, path(:, 1), path(:, 2), '-', 'Color', [1 1 1], 'LineWidth', 3.0);
    plot(ax, path(:, 1), path(:, 2), '-', 'Color', [0.10 0.10 0.10], 'LineWidth', 1.4);
end
draw_markers(ax, env, 2);
end

%% ================= 3-D 集成图（路径 + 采样密集度） =================

function plot_composite_3d(outDir)
modes = {'general_v2', 'narrow', 'suspended', 'ring', 'overhang'};
names = {'General 3-D', 'Narrow 3-D', 'Suspended-obstacle field', ...
    'Cylindrical ring gallery', 'Overhanging platforms'};
algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
bin = 6;
for si = 1:numel(modes)
    env = bair_env3d(modes{si});
    nA = numel(algs);
    fig = figure('Visible', 'off', 'Position', [20 20 1700 1800]);
    for k = 1:nA
        ax = subplot(4, 4, k); hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
        draw_env3d(ax, env);
        R = run_alg_3d(algs{k}, env, modes{si});
        if ~isempty(R.seed) && size(R.seed, 1) >= 2
            plot3(ax, R.seed(:, 1), R.seed(:, 2), R.seed(:, 3), '--', 'Color', [0.92 0.55 0.55], 'LineWidth', 1.0);
        end
        if size(R.path, 1) >= 2
            plot3(ax, R.path(:, 1), R.path(:, 2), R.path(:, 3), '-', 'Color', alg_color(algs{k}), 'LineWidth', 2.8);
            plot3(ax, R.path(:, 1), R.path(:, 2), R.path(:, 3), '.', 'Color', alg_color(algs{k}), 'MarkerSize', 7);
        end
        draw_markers(ax, env, 3);
        axis(ax, [env.bounds(1:2), env.bounds(3:4), env.bounds(5:6)]);
        daspect(ax, [1 1 1]); pbaspect(ax, [1 1 1]);
        set(ax, 'SortMethod', 'childorder');
        view(ax, -37.5, 22);
        if R.ok
            ttl = len_title(algs{k}, R, mean_len_lookup('3d', modes{si}, algs{k}), summary_rank('3d', modes{si}, algs{k}));
        else
            ttl = sprintf('%s  |  no solution', algs{k});
        end
        title(ax, ttl, 'FontSize', 12, 'FontWeight', 'bold', 'Color', alg_color(algs{k}));
        set(ax, 'FontSize', 8);
    end
    for k = 1:nA
        ax = subplot(4, 4, nA + k); hold(ax, 'on'); box(ax, 'on');
        R = run_alg_3d(algs{k}, env, modes{si});
        density_panel_3d(ax, env, R.samples, R.path, bin);
        axis(ax, env.bounds(1:4)); axis(ax, 'equal'); pbaspect(ax, [1 1 1]);
        title(ax, density_title(algs{k}, R, mean_len_lookup('3d', modes{si}, algs{k}), summary_rank('3d', modes{si}, algs{k})), 'FontSize', 12, 'FontWeight', 'bold', ...
            'Color', alg_color(algs{k}));
        set(ax, 'FontSize', 8);
    end
    add_density_colorbar(fig, bin, [0.947 0.055 0.011 0.43]);
    annotation(fig, 'textbox', [0.004 0.815 0.05 0.03], 'String', '(a) final path', ...
        'EdgeColor', 'none', 'FontSize', 13, 'FontWeight', 'bold');
    annotation(fig, 'textbox', [0.004 0.315 0.05 0.03], 'String', '(b) sampling density (x-y projection)', ...
        'EdgeColor', 'none', 'FontSize', 13, 'FontWeight', 'bold');
    sgtitle(fig, sprintf('Eight algorithms under identical seed and budget: %s', names{si}), ...
        'FontSize', 15, 'FontWeight', 'bold');
    save_fig(fig, outDir, sprintf('fig3d_all_%s', modes{si}), false);
    close(fig);
end
end

%% ================= 九算法叠加对比图（三维） =================

function plot_overlay_3d(outDir)
modes = {'general_v2', 'narrow', 'suspended', 'ring', 'overhang'};
names = {'General 3-D', 'Narrow 3-D', 'Suspended obstacles', ...
    'Ring gallery', 'Overhanging platforms'};
algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
hAll = gobjects(1, numel(algs));
axLast = [];
fig = figure('Visible', 'off', 'Position', [30 30 2400 620]);
for si = 1:numel(modes)
    env = bair_env3d(modes{si});
    ax = subplot(1, 5, si); hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
    draw_env3d(ax, env);
    for k = 1:numel(algs)
        R = run_alg_3d(algs{k}, env, modes{si});
        if size(R.path, 1) >= 2
            h = plot3(ax, R.path(:, 1), R.path(:, 2), R.path(:, 3), '-', ...
                'Color', alg_color(algs{k}), 'LineWidth', 2.4);
            if ~isgraphics(hAll(k)), hAll(k) = h; end
        end
    end
    draw_markers(ax, env, 3);
    axis(ax, [env.bounds(1:2), env.bounds(3:4), env.bounds(5:6)]);
    daspect(ax, [1 1 1]); pbaspect(ax, [1 1 1]);
    set(ax, 'SortMethod', 'childorder', 'FontSize', 8);
    view(ax, -37.5, 22);
    title(ax, names{si}, 'FontSize', 11.5, 'FontWeight', 'bold');
    axLast = ax;
end
keep = isgraphics(hAll);
lgd = legend(axLast, hAll(keep), algs(keep), 'Orientation', 'horizontal', 'FontSize', 9, 'Box', 'off');
set(lgd, 'Position', [0.08 0.012 0.84 0.05]);
sgtitle(fig, 'Final paths of all eight algorithms in the same axes (identical seed and budget)', ...
    'FontSize', 14, 'FontWeight', 'bold');
save_fig(fig, outDir, 'fig3d_overlay_all', false);
close(fig);
end

%% ================= 汇总图：八环境改进概览 =================

function plot_summary_figure(outDir)
algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
envs = {{'2d', 'y', '2-D general'}, {'2d', 'n', '2-D narrow'}, {'2d', 'g', '2-D expansive'}, ...
    {'3d', 'general_v2', '3-D general'}, {'3d', 'narrow', '3-D narrow'}, ...
    {'3d', 'suspended', '3-D suspended'}, {'3d', 'ring', '3-D ring'}, {'3d', 'overhang', '3-D overhang'}};
n = numel(envs);
lenAB = nan(1, n); lenBest = nan(1, n); bestName = cell(1, n);
succAB = nan(1, n); succWorst = nan(1, n); worstName = cell(1, n);
for i = 1:n
    dim = envs{i}{1}; sc = envs{i}{2};
    lenAB(i) = summary_lookup(dim, sc, 'AB-IRRT*', 'meanLength');
    succAB(i) = summary_lookup(dim, sc, 'AB-IRRT*', 'successRate');
    bl = inf; bn = ''; ws = inf; wn = '';
    for j = 2:numel(algs)
        v = summary_lookup(dim, sc, algs{j}, 'meanLength');
        if isfinite(v) && v < bl, bl = v; bn = algs{j}; end
        s = summary_lookup(dim, sc, algs{j}, 'successRate');
        if isfinite(s) && s < ws, ws = s; wn = algs{j}; end
    end
    lenBest(i) = bl; bestName{i} = bn; succWorst(i) = ws; worstName{i} = wn;
end
labs = cellfun(@(c) c{3}, envs, 'UniformOutput', false);
fig = figure('Visible', 'off', 'Position', [40 40 1560 580]);
ax = subplot(1, 2, 1);
b = bar(ax, 1:n, [lenAB(:), lenBest(:)], 'grouped'); hold(ax, 'on');
b(1).FaceColor = [0.85 0.08 0.08]; b(2).FaceColor = [0.62 0.62 0.66];
for i = 1:n
    if isfinite(lenAB(i)) && isfinite(lenBest(i))
        d = 100 * (lenAB(i) - lenBest(i)) / lenBest(i);
        yl = max(lenAB(i), lenBest(i));
        text(ax, i, yl * 1.03, sprintf('%+.1f%%', d), 'HorizontalAlignment', 'center', ...
            'FontSize', 8.5, 'Color', [0.85 0.08 0.08], 'FontWeight', 'bold');
    end
end
set(ax, 'XTick', 1:n, 'XTickLabel', labs, 'XTickLabelRotation', 30, 'FontSize', 9);
ylabel(ax, 'Mean path length'); grid(ax, 'on'); ylim(ax, [0, max(lenBest) * 1.18]);
legend(ax, {'AB-IRRT* (proposed)', 'best baseline in that environment'}, ...
    'Location', 'northwest', 'FontSize', 9, 'Box', 'off');
title(ax, '(a) mean path length versus the strongest baseline', 'FontSize', 12, 'FontWeight', 'bold');

ax2 = subplot(1, 2, 2);
% 成功率为 0 时柱子高度为零，看上去像是缺了一根柱，左右两块面板会对不上；
% 这里只给零值柱一个可见的残桩，柱顶标注的仍是真实值 0。
succPlot = succWorst(:);
succPlot(~isfinite(succPlot)) = 0;
succPlot(isfinite(succWorst(:)) & succWorst(:) < 0.05) = 3.0;
b2 = bar(ax2, 1:n, [succAB(:), succPlot], 'grouped'); hold(ax2, 'on');
b2(1).FaceColor = [0.85 0.08 0.08]; b2(2).FaceColor = [0.42 0.42 0.46];
for i = 1:n
    text(ax2, i - 0.16, succAB(i) + 3, sprintf('%.0f', succAB(i)), 'HorizontalAlignment', 'center', ...
        'FontSize', 9, 'Color', [0.85 0.08 0.08], 'FontWeight', 'bold');
    if succWorst(i) < 99.9
        lab = sprintf('%.0f\n(%s)', succWorst(i), worstName{i});
    else
        lab = sprintf('%.0f', succWorst(i));
    end
    text(ax2, i + 0.22, succPlot(i) + 3, lab, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', 'FontSize', 8);
end
ylim(ax2, [0 132]);
set(ax2, 'XTick', 1:n, 'XTickLabel', labs, 'XTickLabelRotation', 30, 'FontSize', 9);
ylabel(ax2, 'Success rate (%)'); grid(ax2, 'on');
legend(ax2, {'AB-IRRT* (proposed)', 'weakest baseline in that environment'}, ...
    'Location', 'northwest', 'FontSize', 9, 'Box', 'off');
title(ax2, '(b) success rate', 'FontSize', 12, 'FontWeight', 'bold');
sgtitle(fig, 'AB-IRRT* versus the strongest and weakest baselines in the eight benchmark environments (50 runs, identical budget)', ...
    'FontSize', 13, 'FontWeight', 'bold');
save_fig(fig, outDir, 'fig_summary_improvement');
close(fig);
end

%% ================= 消融：2-D =================

function plot_ablation_2d(outDir)
entryDir = fileparts(mfilename('fullpath'));
csvFile = fullfile(entryDir, 'ablation_2d.csv');
variants = {'APF-only', 'Informed-RRT*', 'w/o ellipse seeding', 'w/o path-bias sampling', ...
    'w/o dynamic rewiring', 'AB-IRRT* (full)'};
shortNames = {'APF only', 'Informed-RRT*', 'w/o ellipse seed', 'w/o path-bias', ...
    'w/o dyn. rewiring', 'AB-IRRT* (full)'};
[scenes, len, lenStd, succ, firstMs, timeS] = agg_ablation(csvFile, variants);
vcol = [0.45 0.55 0.70; 0.30 0.55 0.80; 0.40 0.65 0.45; 0.75 0.60 0.30; 0.55 0.45 0.70; 0.85 0.08 0.08];

% ---- 平均路径长度 + 成功率 ----
fig = figure('Visible', 'off', 'Position', [40 40 1560 480]);
for i = 1:numel(scenes)
    ax = subplot(1, numel(scenes), i); hold(ax, 'on'); box(ax, 'on');
    y = len(i, :); y(~isfinite(y)) = 0;
    sd = lenStd(i, :); sd(~isfinite(sd) | y == 0) = 0;
    b = bar(ax, y, 0.62, 'FaceColor', 'flat');
    b.CData = vcol;
    errorbar(ax, 1:numel(variants), y, sd, 'k', 'LineStyle', 'none', 'LineWidth', 1.0);
    ymax = max(y + sd);
    ylim(ax, [0, ymax * 1.34 + eps]);
    plot(ax, [0.35, numel(variants) + 0.65], len(i, end) * [1 1], '--', ...
        'Color', [0.85 0.08 0.08], 'LineWidth', 1.2);
    set(ax, 'XTick', 1:numel(variants), 'XTickLabel', shortNames, 'XTickLabelRotation', 28, 'FontSize', 9);
    ylabel(ax, 'Mean path length');
    title(ax, eng_scene(scenes{i}), 'FontSize', 12, 'FontWeight', 'bold');
    for j = 1:numel(variants)
        if ~isfinite(len(i, j))
            text(ax, j, 0.03 * ymax + eps, '0% success', ...
                'HorizontalAlignment', 'center', 'Color', [0.85 0.08 0.08], ...
                'FontWeight', 'bold', 'FontSize', 8.5);
        else
            text(ax, j, y(j) + sd(j) + 0.03 * ymax, sprintf('%.0f%%', succ(i, j)), ...
                'HorizontalAlignment', 'center', 'FontSize', 8.5);
        end
    end
    grid(ax, 'on');
end
sgtitle(fig, ['Ablation study (2-D): mean path length with 1-s.d. bars; text = success rate; ', ...
    'dashed line = full pipeline'], 'FontSize', 13.5, 'FontWeight', 'bold');
save_fig(fig, outDir, 'fig_ablation_2d_len');
close(fig);

% ---- 首解时间 ----
fig = figure('Visible', 'off', 'Position', [40 40 1560 480]);
for i = 1:numel(scenes)
    ax = subplot(1, numel(scenes), i); hold(ax, 'on'); box(ax, 'on');
    y = firstMs(i, :); y(~isfinite(y)) = 0;
    b = bar(ax, y, 0.62, 'FaceColor', 'flat'); b.CData = vcol;
    ylim(ax, [0, max(y) * 1.20 + eps]);
    set(ax, 'XTick', 1:numel(variants), 'XTickLabel', shortNames, 'XTickLabelRotation', 28, 'FontSize', 9);
    ylabel(ax, 'Time to first solution (ms)');
    title(ax, eng_scene(scenes{i}), 'FontSize', 12, 'FontWeight', 'bold');
    grid(ax, 'on');
end
sgtitle(fig, 'Ablation study (2-D): time to first solution (the warm-start effect)', ...
    'FontSize', 13.5, 'FontWeight', 'bold');
save_fig(fig, outDir, 'fig_ablation_2d_first');
close(fig);

% ---- 六变体路径叠加 ----
modes = {'y', 'n', 'g'};
fig = figure('Visible', 'off', 'Position', [40 40 1560 520]);
for si = 1:numel(modes)
    env = bair_env2d(modes{si});
    ax = subplot(1, 3, si); hold(ax, 'on'); box(ax, 'on'); axis(ax, 'equal');
    draw_env2d(ax, env);
    lg = {}; ph = gobjects(1, numel(variants));
    for j = 1:numel(variants)
        R = run_variant_2d(variants{j}, env, modes{si}, si);
        if R.ok && size(R.path, 1) >= 2
            ph(j) = plot(ax, R.path(:, 1), R.path(:, 2), '-', 'Color', vcol(j, :), 'LineWidth', 2.0);
            if isfinite(len(si, j))
                lg{end + 1} = sprintf('%s (mean %.1f)', shortNames{j}, len(si, j)); %#ok<AGROW>
            else
                lg{end + 1} = sprintf('%s (no successful run)', shortNames{j}); %#ok<AGROW>
            end
        end
        draw_markers(ax, env, 2);
    end
    ph = ph(isgraphics(ph));
    axis(ax, env.bounds(1:4)); pbaspect(ax, [1 1 1]);
    title(ax, eng_scene(scenes{si}), 'FontSize', 12, 'FontWeight', 'bold');
    if ~isempty(lg)
        legend(ax, ph, lg, 'Location', 'southoutside', 'Orientation', 'vertical', 'FontSize', 8, 'Box', 'off');
    end
end
sgtitle(fig, ['Ablation study (2-D): final path of each variant (one representative run; ', ...
    'legend gives the 50-run mean length)'], 'FontSize', 13.5, 'FontWeight', 'bold');
save_fig(fig, outDir, 'fig_ablation_2d_paths');
close(fig);
end

function [scenes, len, lenStd, succ, firstMs, timeS] = agg_ablation(csvFile, variants)
T = readtable(csvFile, 'Encoding', 'UTF-8', 'VariableNamingRule', 'preserve');
scenes = unique(T.scenario, 'stable');
nv = numel(variants); ns = numel(scenes);
len = nan(ns, nv); lenStd = nan(ns, nv); succ = nan(ns, nv); firstMs = nan(ns, nv); timeS = nan(ns, nv);
for i = 1:ns
    for j = 1:nv
        mask = strcmp(T.scenario, scenes{i}) & strcmp(T.variant, variants{j});
        if ~any(mask), continue; end
        ok = logical(T.success(mask));
        L = double(T.length(mask)); L = L(ok & isfinite(L));
        if ~isempty(L), len(i, j) = mean(L); lenStd(i, j) = std(L); end
        succ(i, j) = 100 * mean(ok);
        f = double(T.firstSolutionTime(mask)); f = f(ok & isfinite(f));
        if ~isempty(f), firstMs(i, j) = 1000 * mean(f); end
        t = double(T.time(mask)); t = t(ok & isfinite(t));
        if ~isempty(t), timeS(i, j) = mean(t); end
    end
end
end

function R = run_variant_2d(variant, env, mode, ci)
o = struct('maxIter', 2000, 'step', 4, 'goalBias', 0.08, 'rewireRadius', 14, ...
    'rewireMax', 16, 'pbias', 0.5);
persistent CV
if isempty(CV), CV = containers.Map('KeyType', 'char', 'ValueType', 'any'); end
vkey = sprintf('%s|%s', mode, variant);
if isKey(CV, vkey), R = CV(vkey); return; end
cf = fig_cache_path('abl2d', mode, variant, 2000);
if exist(cf, 'file'), S = load(cf); R = S.R; CV(vkey) = R; return; end
rng(seed_for_figure_run('abl2d', mode), 'twister');
[seedPath, ai] = stable_apf2d(env, struct('useFallback', true, 'step', o.step));
seed = seedPath; if ~ai.success, seed = []; end
ro = struct('maxIter', o.maxIter, 'step', o.step, 'goalBias', o.goalBias, ...
    'rewireRadius', o.rewireRadius, 'pbias', 0.0, 'dynamicRewire', false, ...
    'rewireMax', o.rewireMax, 'useSeedEllipse', true);
switch variant
    case 'APF-only'
        [p, i] = stable_apf2d(env, struct('useFallback', false, 'step', o.step));
    case 'Informed-RRT*'
        [p, i] = informed_rrtstar2d(env, [], ro);
    case 'w/o ellipse seeding'
        ro.pbias = o.pbias; ro.useSeedEllipse = false;
        [p, i] = informed_rrtstar2d(env, seed, ro);
    case 'w/o path-bias sampling'
        ro.pbias = 0; ro.useSeedEllipse = true;
        [p, i] = informed_rrtstar2d(env, seed, ro);
    case 'w/o dynamic rewiring'
        ro.pbias = o.pbias; ro.useSeedEllipse = true; ro.dynamicRewire = false;
        ro.rewireRadius = o.rewireMax;
        [p, i] = informed_rrtstar2d(env, seed, ro);
    case 'AB-IRRT* (full)'
        ro.pbias = o.pbias; ro.useSeedEllipse = true; ro.dynamicRewire = true;
        [p, i] = informed_rrtstar2d(env, seed, ro);
    otherwise
        error('unknown variant %s', variant);
end
R = struct('path', p, 'ok', i.success, 'L', NaN);
if size(p, 1) >= 2, R.L = sum(sqrt(sum(diff(p).^2, 2))); end
CV(vkey) = R;
save(cf, 'R');
end

%% ================= 消融：3-D =================

function plot_ablation_3d(outDir)
entryDir = fileparts(mfilename('fullpath'));
csvFile = fullfile(entryDir, 'ablation_3d.csv');
variants = {'APF-only', 'Informed-RRT*', 'AB-IRRT* (full)'};
shortNames = {'APF only', 'Informed-RRT*', 'AB-IRRT* (full)'};
[scenes, len, lenStd, succ, firstMs] = agg_ablation(csvFile, variants);
vcol = [0.45 0.55 0.70; 0.30 0.55 0.80; 0.85 0.08 0.08];

fig = figure('Visible', 'off', 'Position', [40 40 1200 900]);
for i = 1:numel(scenes)
    ax = subplot(2, numel(scenes), i); hold(ax, 'on'); box(ax, 'on');
    y = len(i, :); y(~isfinite(y)) = 0;
    sd = lenStd(i, :); sd(~isfinite(sd) | y == 0) = 0;
    b = bar(ax, y, 0.6, 'FaceColor', 'flat'); b.CData = vcol;
    errorbar(ax, 1:numel(variants), y, sd, 'k', 'LineStyle', 'none', 'LineWidth', 1.0);
    ylim(ax, [0, max(y + sd) * 1.28 + eps]);
    set(ax, 'XTick', 1:numel(variants), 'XTickLabel', shortNames, 'FontSize', 9.5);
    ylabel(ax, 'Mean path length');
    title(ax, sprintf('%s | path length', eng_scene(scenes{i})), 'FontSize', 12, 'FontWeight', 'bold');
    for j = 1:numel(variants)
        yv = y(j);
        if yv > 0
            text(ax, j, yv + sd(j) + 0.02 * max(y + sd), sprintf('%.0f%%', succ(i, j)), ...
                'HorizontalAlignment', 'center', 'FontSize', 9);
        else
            text(ax, j, 0.03 * max(y + sd) + eps, '0% success', 'HorizontalAlignment', 'center', ...
                'Color', [0.85 0.08 0.08], 'FontWeight', 'bold', 'FontSize', 8.5);
        end
    end
    grid(ax, 'on');

    ax2 = subplot(2, numel(scenes), numel(scenes) + i); hold(ax2, 'on'); box(ax2, 'on');
    y2 = firstMs(i, :); y2(~isfinite(y2)) = 0;
    b2 = bar(ax2, y2, 0.6, 'FaceColor', 'flat'); b2.CData = vcol;
    ylim(ax2, [0, max(y2) * 1.20 + eps]);
    set(ax2, 'XTick', 1:numel(variants), 'XTickLabel', shortNames, 'FontSize', 9.5);
    ylabel(ax2, 'Time to first solution (ms)');
    title(ax2, sprintf('%s | first solution', eng_scene(scenes{i})), 'FontSize', 12, 'FontWeight', 'bold');
    grid(ax2, 'on');
end
sgtitle(fig, 'Ablation study (3-D): mean path length and time to first solution', ...
    'FontSize', 13.5, 'FontWeight', 'bold');
save_fig(fig, outDir, 'fig_ablation_3d');
close(fig);
end
