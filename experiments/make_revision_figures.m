function make_revision_figures(sel)
%MAKE_REVISION_FIGURES 生成修订稿论文图（无工具箱依赖）。
%
% 可选参数 sel（默认 'all'）只重出部分图：
%   make_revision_figures('bars2d')        只出 2D 成功率/运行时间/首解时间柱状图
%   make_revision_figures({'bars2d','bars3d'})
%   可用名：boxes2d bars2d boxes3d bars3d anytime2d anytime3d sampling2d scenes3d
%
% 输出目录：<repo>/figures
% 内容：
%   fig2d_box_*.png/pdf    2D 路径长度箱线图（每场景）
%   fig2d_success.png      2D 成功率柱状图
%   fig2d_time.png         2D 运行时间（均值±标准差）
%   fig2d_first.png        2D 首解时间（均值±标准差）
%   fig2d_sampling_*.png   2D 采样点云对比（RRT*/IRRT*/AB-IRRT*）
%   fig2d_anytime_*.png    2D anytime 收敛曲线（中位数）
%   fig3d_box_*.png        3D 路径长度箱线图（每场景）
%   fig3d_success.png      3D 成功率柱状图
%   fig3d_time.png / fig3d_first.png
%   fig3d_anytime_*.png    3D anytime 曲线
%   fig3d_scene_*.png      3D 场景渲染 + 示例路径
%
% 依赖实验输出：results_2d_v2.csv / anytime_2d_v2.mat /
%               results_3d_v2*.csv / anytime_3d_v2*.mat

entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
wsRoot = fileparts(rootDir);
outDir = fullfile(rootDir, 'figures');
if ~exist(outDir, 'dir'), mkdir(outDir); end
addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));

% ---- 读取 2D / 3D 结果 ----
T2 = readtable(fullfile(entryDir, 'results_2d_v2.csv'), 'Encoding', 'UTF-8');
T3 = table();
files = dir(fullfile(entryDir, 'results_3d_v2*.csv'));
for f = 1:numel(files)
    nm = files(f).name;
    if contains(nm, 'test'), continue; end
    Ti = readtable(fullfile(entryDir, nm), 'Encoding', 'UTF-8');
    if isempty(T3), T3 = Ti; else, T3 = [T3; Ti]; end %#ok<AGROW>
end

algOrder = {'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only', 'AB-IRRT*'};

if nargin < 1 || isempty(sel), sel = {'all'}; end
if ischar(sel), sel = {sel}; end
want = @(s) any(strcmp(sel, 'all')) || any(strcmp(sel, s));

% ---- 2D 图 ----
if want('boxes2d'), make_box_figs(T2, algOrder, outDir, '2D'); end
if want('bars2d'), make_bar_figs(T2, algOrder, outDir, '2D'); end

% ---- 3D 图 ----
if ~isempty(T3)
    if want('boxes3d'), make_box_figs(T3, algOrder, outDir, '3D'); end
    if want('bars3d'), make_bar_figs(T3, algOrder, outDir, '3D'); end
end

% ---- anytime 曲线 ----
if want('anytime2d') && exist(fullfile(entryDir, 'anytime_2d_v2.mat'), 'file')
    plot_anytime(fullfile(entryDir, 'anytime_2d_v2.mat'), ...
        {'广阔', '狭窄'}, algOrder, outDir, '2D');
end
filesAt = dir(fullfile(entryDir, 'anytime_3d_v2*.mat'));
for f = 1:numel(filesAt)
    if contains(filesAt(f).name, 'test'), continue; end
    if ~want('anytime3d'), continue; end
    S = load(fullfile(entryDir, filesAt(f).name));
    sc = unique({S.anytime.scenario});
    [~, base] = fileparts(filesAt(f).name);
    tagName = regexprep(base, '^anytime_3d_v2_?', '');
    plot_anytime(fullfile(entryDir, filesAt(f).name), sc, algOrder, outDir, '3D', tagName);
end

% ---- 2D 采样可视化与 3D 场景渲染 ----
if want('sampling2d'), visualize_sampling_2d(outDir); end
if want('scenes3d'), visualize_scenes_3d(outDir); end

fprintf('Figures written to %s\n', outDir);
end

%% ================= 通用绘图工具 =================

function c = alg_color(name)
% 与 make_algorithm_figures / make_convergence_figure / make_extra_figures / make_pareto_figure
% 保持完全一致的算法配色（否则同一篇稿子里 PRM*/FMT* 的颜色会互换）
switch name
    case 'AB-IRRT*', c = [0.85 0.08 0.08];
    case 'RRT*', c = [0.25 0.25 0.28];
    case 'Informed-RRT*', c = [0.08 0.35 0.88];
    case 'RRT-Connect', c = [0.05 0.62 0.28];
    case 'PRM*', c = [0.55 0.12 0.72];
    case 'FMT*', c = [0.95 0.55 0.05];
    case 'BIT*', c = [0.05 0.60 0.72];
    case 'RRT#', c = [0.88 0.25 0.62];
    case 'APF-only', c = [0.55 0.55 0.05];
    otherwise, c = [0.4 0.4 0.4];
end
end

function out = eng_names(in)
%ENG_NAMES 场景名 -> 英文（图中题注/图例统一使用英文）。
%   输入 char -> 返回 char；输入 cellstr -> 返回 cellstr。
key = {'一般', '狭窄', '广阔', '悬空', '环廊', '悬挑'};
val = {'General', 'Narrow passage', 'Expansive', 'Suspended obstacles', ...
    'Ring gallery', 'Overhanging platforms'};
if ischar(in)
    names = {in}; single = true;
else
    names = in; single = false;
end
for i = 1:numel(names)
    k = find(strcmp(key, names{i}), 1);
    if ~isempty(k), names{i} = val{k}; end
end
if single
    out = names{1};
else
    out = names;
end
end

function save_fig(fig, outDir, name)
% print + 显式纸面尺寸：保证导出图与屏幕版式等比（exportgraphics 会拉伸版式）。
w = fig.Position(3); h = fig.Position(4);
set(fig, 'PaperUnits', 'inches', 'PaperPositionMode', 'manual', ...
    'PaperPosition', [0 0 w / 96, h / 96]);
print(fig, fullfile(outDir, [name, '.png']), '-dpng', '-r200');
try
    print(fig, fullfile(outDir, [name, '.pdf']), '-dpdf');
catch
    % PDF 导出失败时忽略（保留 PNG）
end
end

function draw_box(ax, x, data, color)
% 手工绘制箱线图（避免依赖 Statistics Toolbox）。
if isempty(data) || all(~isfinite(data)), return; end
data = data(isfinite(data));
q = quantiles_local(data, [25 50 75]);
iqr = q(3) - q(1);
lo = min(data(data >= q(1) - 1.5 * iqr));
hi = max(data(data <= q(3) + 1.5 * iqr));
w = 0.28;
axes(ax); hold(ax, 'on');
patch(ax, x + w * [-1 1 1 -1], [q(1) q(1) q(3) q(3)], color, ...
    'FaceAlpha', 0.35, 'EdgeColor', color, 'LineWidth', 1.0);
plot(ax, [x x], [q(3) hi], '-', 'Color', color, 'LineWidth', 1.0);
plot(ax, [x x], [lo q(1)], '-', 'Color', color, 'LineWidth', 1.0);
plot(ax, x + 0.12 * [-1 1], [lo lo], '-', 'Color', color, 'LineWidth', 1.0);
plot(ax, x + 0.12 * [-1 1], [hi hi], '-', 'Color', color, 'LineWidth', 1.0);
plot(ax, x, q(2), 'o', 'MarkerSize', 4, 'MarkerFaceColor', color, 'MarkerEdgeColor', 'none');
out = data(data < lo | data > hi);
plot(ax, x * ones(size(out)), out, '.', 'Color', color, 'MarkerSize', 3);
end

function q = quantiles_local(x, p)
x = sort(x(:));
q = zeros(size(p));
for i = 1:numel(p)
    rank = (p(i) / 100) * (numel(x) - 1) + 1;
    lo = floor(rank); hi = ceil(rank);
    if lo == hi
        q(i) = x(lo);
    else
        w = rank - lo;
        q(i) = x(lo) * (1 - w) + x(hi) * w;
    end
end
end

function make_box_figs(T, algOrder, outDir, tag)
scenes = unique(T.scenario, 'stable');
algs = intersect(algOrder, unique(T.algorithm, 'stable'), 'stable');
for i = 1:numel(scenes)
    fig = figure('Visible', 'off', 'Position', [100 100 900 420]);
    ax = axes(fig); hold(ax, 'on'); box(ax, 'on');
    for j = 1:numel(algs)
        mask = strcmp(T.scenario, scenes{i}) & strcmp(T.algorithm, algs{j}) & logical(T.success);
        draw_box(ax, j, T.length(mask), alg_color(algs{j}));
    end
    set(ax, 'XTick', 1:numel(algs), 'XTickLabel', algs, 'XTickLabelRotation', 30);
    ylabel(ax, 'Path length');
    title(ax, sprintf('%s path-length distribution: %s', tag, eng_names(scenes{i})));
    grid(ax, 'on');
    save_fig(fig, outDir, sprintf('fig%s_box_%d', lower(tag), i));
    close(fig);
end
% ---- 集成图：所有场景并排（共享纵轴） ----
fig = figure('Visible', 'off', 'Position', [40 40 max(560, 430 * numel(scenes)) 480]);
for i = 1:numel(scenes)
    ax = subplot(1, numel(scenes), i); hold(ax, 'on'); box(ax, 'on');
    for j = 1:numel(algs)
        mask = strcmp(T.scenario, scenes{i}) & strcmp(T.algorithm, algs{j}) & logical(T.success);
        draw_box(ax, j, T.length(mask), alg_color(algs{j}));
    end
    set(ax, 'XTick', 1:numel(algs), 'XTickLabel', algs, 'XTickLabelRotation', 40, 'FontSize', 8);
    ylabel(ax, 'Path length');
    title(ax, eng_names(scenes{i}), 'FontSize', 11.5, 'FontWeight', 'bold');
    grid(ax, 'on');
    if i > 1, set(ax, 'YTickLabel', {}); end
end
sgtitle(fig, sprintf('%s path-length distributions over 50 paired runs (identical budget)', tag), ...
    'FontSize', 13.5, 'FontWeight', 'bold');
save_fig(fig, outDir, sprintf('fig%s_box_all', lower(tag)));
close(fig);
end

function make_bar_figs(T, algOrder, outDir, tag)
% 环境按稿件里的固定顺序排列（否则会按文件名/字母序，变成 general, narrow, overhang, ring, suspended）
allSc = unique(T.scenario, 'stable');
pref = {'一般', '狭窄', '广阔', '悬空', '环廊', '悬挑'};
scenes = [pref(ismember(pref, allSc)), allSc(~ismember(allSc, pref))];
algs = intersect(algOrder, unique(T.algorithm, 'stable'), 'stable');
nA = numel(algs); nS = numel(scenes);
success = zeros(nA, nS); tmean = zeros(nA, nS); tci = zeros(nA, nS, 2);
fmean = zeros(nA, nS); fci = zeros(nA, nS, 2);
for i = 1:nS
    for j = 1:nA
        mask = strcmp(T.scenario, scenes{i}) & strcmp(T.algorithm, algs{j});
        success(j, i) = 100 * mean(logical(T.success(mask)));
        tt = T.time(mask & logical(T.success));
        if numel(tt) >= 2
            tmean(j, i) = mean(tt); s = std(tt);
            tci(j, i, :) = [max(tmean(j, i) - s, tmean(j, i) / 5), tmean(j, i) + s];
        elseif ~isempty(tt)
            tmean(j, i) = mean(tt); tci(j, i, :) = [NaN NaN];
        else, tmean(j, i) = NaN; end
        ff = T.firstSolutionTime(mask & logical(T.success));
        ff = ff(isfinite(ff));
        if numel(ff) >= 2
            fmean(j, i) = mean(ff); s = std(ff);
            fci(j, i, :) = [max(fmean(j, i) - s, fmean(j, i) / 5), fmean(j, i) + s];
        elseif ~isempty(ff)
            fmean(j, i) = mean(ff); fci(j, i, :) = [NaN NaN];
        else, fmean(j, i) = NaN; end
    end
end

% 时间一律换算为毫秒（避免毫秒级算法在图中被压成 0）
tmean = 1000 * tmean; tci = 1000 * tci;
fmean = 1000 * fmean; fci = 1000 * fci;

% 成功率（成功率为 0 的组合画成最小可见高度，避免看上去像是缺了柱子，真实值见表格）
successPlot = success;
successPlot(successPlot <= 0.05) = 1;
fig = figure('Visible', 'off', 'Position', [100 100 980 420]);
ax = axes(fig); b = bar(ax, successPlot, 'grouped'); hold(ax, 'on');
color_env_series(b, scenes);
set(ax, 'XTick', 1:nA, 'XTickLabel', algs, 'XTickLabelRotation', 30);
ylabel(ax, 'Success rate (%)'); ylim(ax, [0 105]);
legend(ax, eng_names(scenes), 'Location', 'southoutside', 'Orientation', 'horizontal', 'NumColumns', 3);
title(ax, sprintf('%s success rate', tag)); grid(ax, 'on');
save_fig(fig, outDir, sprintf('fig%s_success', lower(tag))); close(fig);

% 运行时间（均值±标准差）
fig = figure('Visible', 'off', 'Position', [100 100 980 420]);
ax = axes(fig); b = bar(ax, tmean, 'grouped'); hold(ax, 'on');
color_env_series(b, scenes);
for k = 1:numel(b)
    for i = 1:nA
        xc = b(k).XEndPoints(i);
        lo = tci(i, k, 1); hi = tci(i, k, 2);
        if isfinite(lo) && isfinite(hi)
            plot(ax, [xc xc], [lo hi], 'k-', 'LineWidth', 0.8);
            plot(ax, xc + 0.02 * [-1 1], [lo lo], 'k-', 'LineWidth', 0.8);
            plot(ax, xc + 0.02 * [-1 1], [hi hi], 'k-', 'LineWidth', 0.8);
        end
    end
end
set(ax, 'XTick', 1:nA, 'XTickLabel', algs, 'XTickLabelRotation', 30);
% 毫秒级与万毫秒级共存，改对数轴否则小值全被压成 0
set(ax, 'YScale', 'log');
lo0 = min(tmean(tmean > 0)) / 1.6; hi0 = max(tmean(isfinite(tmean))) * 1.8;
if isfinite(lo0) && isfinite(hi0) && lo0 < hi0, ylim(ax, [lo0 hi0]); end
ylabel(ax, 'Runtime (ms, log scale)');
legend(ax, eng_names(scenes), 'Location', 'southoutside', 'Orientation', 'horizontal', 'NumColumns', 3);
title(ax, sprintf('%s runtime (mean \\pm std)', tag)); grid(ax, 'on');
save_fig(fig, outDir, sprintf('fig%s_time', lower(tag))); close(fig);

% 首解时间
fig = figure('Visible', 'off', 'Position', [100 100 980 420]);
ax = axes(fig); b = bar(ax, fmean, 'grouped'); hold(ax, 'on');
color_env_series(b, scenes);
for k = 1:numel(b)
    for i = 1:nA
        xc = b(k).XEndPoints(i);
        lo = fci(i, k, 1); hi = fci(i, k, 2);
        if isfinite(lo) && isfinite(hi)
            plot(ax, [xc xc], [lo hi], 'k-', 'LineWidth', 0.8);
        end
    end
end
set(ax, 'XTick', 1:nA, 'XTickLabel', algs, 'XTickLabelRotation', 30);
set(ax, 'YScale', 'log');
lo0 = min(fmean(fmean > 0)) / 1.6; hi0 = max(fmean(isfinite(fmean))) * 1.8;
if isfinite(lo0) && isfinite(hi0) && lo0 < hi0, ylim(ax, [lo0 hi0]); end
ylabel(ax, 'Time to first solution (ms, log scale)');
legend(ax, eng_names(scenes), 'Location', 'southoutside', 'Orientation', 'horizontal', 'NumColumns', 3);
title(ax, sprintf('%s time to first solution', tag)); grid(ax, 'on');
save_fig(fig, outDir, sprintf('fig%s_first', lower(tag))); close(fig);
end

function color_env_series(b, scenes)
% 柱状图按“环境”配色：bar 返回的每个对象对应一个环境系列（X 轴是算法）。
% 环境色板与算法色板相互独立；图例（eng_names(scenes)）的色块因此与实际颜色一致。
for k = 1:numel(b)
    b(k).FaceColor = env_color(scenes{min(k, numel(scenes))});
    b(k).EdgeColor = [0.15 0.15 0.15];
end
end

function c = env_color(name)
%ENV_COLOR 环境配色（仅用于“同一张图里区分环境”的柱状图；不要与算法配色混用）
switch name
    case '一般',                   c = [0.40 0.76 0.65];   % General
    case '狭窄',                   c = [0.99 0.55 0.38];   % Narrow passage
    case '广阔',                   c = [0.55 0.63 0.80];   % Expansive
    case '悬空',                   c = [0.91 0.54 0.76];   % Suspended obstacles
    case '环廊',                   c = [0.65 0.85 0.33];   % Ring gallery
    case '悬挑',                   c = [1.00 0.85 0.18];   % Overhanging platforms
    otherwise,                         c = [0.60 0.60 0.60];
end
end

function plot_anytime(matFile, scenarios, algOrder, outDir, tag, nameTag)
%PLOT_ANYTIME anytime 收敛曲线（两块面板，共享横轴）
%   (a) 固定运行集上的中位代价：每次运行的代价曲线取累积最小；运行结束后沿用其最终
%       代价；到该时刻仍未解出的运行记作 +inf。这样每个时刻都是同一批运行在比较，
%       中位数曲线单调不增（旧画法用 omitnan，参与统计的运行集合随时间变化，会出现
%       “后面反而更长”“曲线半途断掉”等伪迹）。
%   (b) 已解比例（%）：该时刻已有可行解的运行占比，直接显示“时间/采样影响成功率”。
if nargin < 6, nameTag = ''; end
S = load(matFile);
A = S.anytime;
if isempty(A), return; end
algs = intersect(algOrder, unique({A.algorithm}), 'stable');
for i = 1:numel(scenarios)
    sc = scenarios{i};
    mask = strcmp({A.scenario}, sc);
    if ~any(mask), continue; end
    sub = A(mask);
    Tmax = 0;
    for k = 1:numel(sub)
        t = sub(k).t;
        if ~isempty(t), Tmax = max(Tmax, max(t(isfinite(t)))); end
    end
    if ~isfinite(Tmax) || Tmax <= 0, continue; end
    tGrid = linspace(0, Tmax, 240);
    fig = figure('Visible', 'off', 'Position', [100 100 760 560]);
    ax = subplot(4, 1, [1 2 3]); hold(ax, 'on'); box(ax, 'on');
    ax2 = subplot(4, 1, 4); hold(ax2, 'on'); box(ax2, 'on');
    lg = {};
    for j = 1:numel(algs)
        alg = algs{j};
        sel = strcmp({sub.algorithm}, alg);
        runs = sub(sel);
        nRun = numel(runs);
        if nRun == 0, continue; end
        C = inf(nRun, numel(tGrid));      % 固定运行集：未解出 = +inf
        firstT = nan(1, nRun);
        for k = 1:nRun
            t = runs(k).t; c = runs(k).cost;
            if isempty(t), continue; end
            keep = isfinite(t) & isfinite(c);
            t = t(keep); c = c(keep);
            if isempty(t), continue; end
            [t, ia] = unique(t); c = c(ia);
            c = cummin(c);
            firstT(k) = t(1);
            if numel(t) == 1
                ct = inf(1, numel(tGrid));
                ct(tGrid >= t(1)) = c(1);
            else
                ct = interp1(t, c, tGrid, 'previous');
                ct(tGrid >= t(end)) = c(end);   % 运行已结束：之后沿用最终代价
                ct(tGrid < t(1)) = inf;
                ct(isnan(ct)) = inf;
            end
            C(k, :) = ct;
        end
        med = median(C, 1);                % 固定运行集的中位数 → 单调不增
        valid = isfinite(med);
        if ~any(valid), continue; end
        plot(ax, tGrid(valid), med(valid), '-', 'Color', alg_color(alg), 'LineWidth', 1.6);
        frac = 100 * mean(tGrid(:)' >= firstT(:), 1);
        plot(ax2, tGrid, frac, '-', 'Color', alg_color(alg), 'LineWidth', 1.2);
        lg{end + 1} = alg; %#ok<AGROW>
    end
    ylabel(ax, 'Best cost (median over the 50 runs)');
    title(ax, sprintf('%s anytime convergence: %s', tag, eng_names(sc)));
    if ~isempty(lg), legend(ax, lg, 'Location', 'northeast'); end
    grid(ax, 'on');
    ylabel(ax2, 'Solved (%)'); xlabel(ax2, 'Time (s)');
    ylim(ax2, [0 105]); grid(ax2, 'on');
    if isempty(nameTag)
        save_fig(fig, outDir, sprintf('fig%s_anytime_%d', lower(tag), i));
    else
        save_fig(fig, outDir, sprintf('fig%s_anytime_%s', lower(tag), nameTag));
    end
    close(fig);
end
end

%% ================= 采样可视化（2D） =================

function visualize_sampling_2d(outDir)
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(fullfile(rootDir, 'algorithms_2d'));
scenes = {'y', 'g'};
titles = {'General 2-D environment', 'Expansive 2-D environment'};
budgets = [1200, 1200];
for si = 1:numel(scenes)
    env = bair_env2d(scenes{si});
    o = struct('maxIter', budgets(si), 'step', 4, 'goalBias', 0.08, 'rewireRadius', 14, ...
        'pbias', 0.5, 'dynamicRewire', true, 'rewireMax', 16);
    fig = figure('Visible', 'off', 'Position', [60 60 1400 420]);
    % RRT*
    rng(11, 'twister'); [p1, i1] = rrtstar2d(env, o);
    % Informed-RRT*
    rng(11, 'twister'); [p2, i2] = informed_rrtstar2d(env, [], o);
    % AB-IRRT*
    rng(11, 'twister'); [~, ~, d3] = bair_core2d(scenes{si}, o); p3 = d3.path;

    sets = {{p1, i1, 'RRT*'}, {p2, i2, 'Informed-RRT*'}, {p3, [], 'AB-IRRT*'}};
    for k = 1:3
        ax = subplot(1, 3, k); hold(ax, 'on'); axis(ax, 'equal'); box(ax, 'on');
        draw_env2d(ax, env);
        pk = sets{k}{1};
        if k <= 2 && isfield(sets{k}{2}, 'samples')
            Smp = sets{k}{2}.samples;
            if size(Smp, 1) > 1500
                idx = randperm(size(Smp, 1), 1500);
                Smp = Smp(idx, :);
            end
            plot(ax, Smp(:, 1), Smp(:, 2), '.', 'Color', [0.55 0.65 0.85], 'MarkerSize', 3);
        elseif k == 3
            apf = d3.apfPath;
            if ~isempty(apf), plot(ax, apf(:, 1), apf(:, 2), '--', 'Color', [0.85 0.45 0.45], 'LineWidth', 1.0); end
        end
        if size(pk, 1) >= 2
            plot(ax, pk(:, 1), pk(:, 2), '-', 'Color', [0.85 0.10 0.10], 'LineWidth', 1.8);
        end
        plot(ax, env.start(1), env.start(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
        plot(ax, env.goal(1), env.goal(2), 'ks', 'MarkerFaceColor', 'g', 'MarkerSize', 5);
        axis(ax, [env.bounds(1:2) env.bounds(3:4)]);
        title(ax, sets{k}{3});
    end
    sgtitle(fig, titles{si});
    save_fig(fig, outDir, sprintf('fig2d_sampling_%s', scenes{si}));
    close(fig);
end
end

function draw_env2d(ax, env)
for k = 1:size(env.squareAll, 1)
    r = env.squareAll(k, :);
    patch(ax, r(1) + [0 r(3) r(3) 0], r(2) + [0 0 r(4) r(4)], [0.25 0.25 0.25], ...
        'EdgeColor', 'none');
end
for k = 1:size(env.round, 1)
    c = env.round(k, :);
    th = linspace(0, 2 * pi, 40);
    patch(ax, c(1) + c(3) * cos(th), c(2) + c(3) * sin(th), [0.25 0.25 0.25], 'EdgeColor', 'none');
end
end

%% ================= 3D 场景渲染 =================

function visualize_scenes_3d(outDir)
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(fullfile(rootDir, 'algorithms_3d'));
modes = {'general_v2', 'narrow', 'suspended', 'ring', 'overhang'};
titles = {'General', 'Narrow (legacy)', 'Suspended obstacles', 'Cylindrical ring gallery', 'Overhanging platforms'};
budget = 6000;
for mi = 1:numel(modes)
    env = bair_env3d(modes{mi});
    bo = struct('maxIter', budget, 'step', 6, 'goalBias', 0.08, 'rewireRadius', 18, ...
        'pbias', 0.1, 'dynamicRewire', true, 'rewireMax', 30, 'useFallback', true);
    rng(5, 'twister');
    [~, ~, d] = bair_core3d(modes{mi}, bo);
    rng(5, 'twister');
    o3 = struct('maxIter', budget, 'step', 6, 'goalBias', 0.08, 'rewireRadius', 18);
    [pR, iR] = rrtstar3d(env, o3);
    rng(5, 'twister');
    [pI, iI] = informed_rrtstar3d(env, [], o3);

    fig = figure('Visible', 'off', 'Position', [80 80 1150 780]);
    ax = axes(fig); hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
    draw_env3d(ax, env);
    hp = gobjects(1, 3); lg = {};
    if iR.success && size(pR, 1) >= 2
        hp(1) = plot3(ax, pR(:, 1), pR(:, 2), pR(:, 3), '-', 'Color', [0.20 0.20 0.22], 'LineWidth', 2.4);
        lg{end + 1} = 'RRT*';
    end
    if iI.success && size(pI, 1) >= 2
        hp(2) = plot3(ax, pI(:, 1), pI(:, 2), pI(:, 3), '-', 'Color', [0.08 0.35 0.88], 'LineWidth', 2.4);
        lg{end + 1} = 'Informed-RRT*';
    end
    if size(d.path, 1) >= 2
        hp(3) = plot3(ax, d.path(:, 1), d.path(:, 2), d.path(:, 3), '-', 'Color', [0.85 0.10 0.10], 'LineWidth', 3.2);
        lg{end + 1} = 'AB-IRRT* (proposed)';
    end
    plot3(ax, env.start(1), env.start(2), env.start(3), 'o', 'MarkerSize', 9, ...
        'MarkerFaceColor', [0 0 0], 'MarkerEdgeColor', [1 1 1], 'LineWidth', 1.0);
    plot3(ax, env.goal(1), env.goal(2), env.goal(3), 'p', 'MarkerSize', 16, ...
        'MarkerFaceColor', [0.05 0.75 0.20], 'MarkerEdgeColor', [0 0 0], 'LineWidth', 1.0);
    axis(ax, [env.bounds(1:2), env.bounds(3:4), env.bounds(5:6)]);
    set(ax, 'SortMethod', 'childorder', 'FontSize', 10);
    xlabel(ax, 'x'); ylabel(ax, 'y'); zlabel(ax, 'z');
    title(ax, sprintf('%s | final paths of three planners', titles{mi}), 'FontSize', 12.5, 'FontWeight', 'bold');
    view(ax, -37.5, 25);
    hp = hp(isgraphics(hp));
    legend(ax, hp, lg, 'Location', 'northeastoutside', 'FontSize', 10, 'Box', 'off');
    save_fig(fig, outDir, sprintf('fig3d_scene_%s', modes{mi}));
    close(fig);
end
end

function draw_env3d(ax, env)
% 障碍物半透明 + 暗色描边；配合 SortMethod='childorder' 保证路径不会被盖住。
for k = 1:numel(env.cube.axisX)
    lo = [env.cube.axisX(k), env.cube.axisY(k), env.cube.axisZ(k)];
    ln = [env.cube.lengthx(k), env.cube.lengthy(k), env.cube.lengthz(k)];
    draw_cube(ax, lo, ln);
end
for k = 1:numel(env.cylinder.X)
    R = env.cylinder.radius(k); H = env.cylinder.lengthZ(k);
    [cx, cy, cz] = cylinder(R, 32);
    surf(ax, cx + env.cylinder.X(k), cy + env.cylinder.Y(k), cz * H + env.cylinder.Z(k), ...
        'FaceColor', [0.32 0.32 0.35], 'EdgeColor', 'none', 'FaceAlpha', 0.42);
end
for k = 1:numel(env.sphere.X)
    [sx, sy, sz] = sphere(28);
    R = env.sphere.radius(k);
    surf(ax, sx * R + env.sphere.X(k), sy * R + env.sphere.Y(k), sz * R + env.sphere.Z(k), ...
        'FaceColor', [0.40 0.40 0.43], 'EdgeColor', 'none', 'FaceAlpha', 0.42);
end
end

function draw_cube(ax, lo, ln)
P = [lo(1) lo(2) lo(3); lo(1) + ln(1) lo(2) lo(3); lo(1) + ln(1) lo(2) + ln(2) lo(3); lo(1) lo(2) + ln(2) lo(3); ...
     lo(1) lo(2) lo(3) + ln(3); lo(1) + ln(1) lo(2) lo(3) + ln(3); lo(1) + ln(1) lo(2) + ln(2) lo(3) + ln(3); lo(1) lo(2) + ln(2) lo(3) + ln(3)];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch(ax, 'Vertices', P, 'Faces', F, 'FaceColor', [0.32 0.32 0.35], ...
    'EdgeColor', [0.05 0.05 0.05], 'LineWidth', 0.5, 'FaceAlpha', 0.40);
end
