function results = run_ablation2d(runs, opts)
%RUN_ABLATION2D AB-IRRT* 机制消融实验（二维，6 变体 × 3 场景 × runs 次）。
%
% 变体：
%   APF-only                仅局部预规划（无全局优化阶段）
%   Informed-RRT*           纯全局采样（无预规划输入）
%   w/o ellipse seeding     使用预规划路径做偏置采样，但不初始化采样椭圆
%   w/o path-bias sampling  使用椭圆种子，但关闭预规划路径点偏置采样
%   w/o dynamic rewiring    使用种子与偏置，重布线半径固定为 r_max
%   AB-IRRT* (full)         完整管线（椭圆种子 + 路径偏置 + 动态重布线）
%
% 输出：ablation_2d.csv（UTF-8）；列与 run_2d_comparison 保持一致的部分字段。

if nargin < 1 || isempty(runs), runs = 50; end
if nargin < 2, opts = struct; end
def = struct('maxIter', 2000, 'step', 4, 'goalBias', 0.08, 'rewireRadius', 14, ...
    'rewireMax', 16, 'pbias', 0.5, 'outputFile', '', 'quiet', false);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));
if isempty(opts.outputFile), opts.outputFile = fullfile(entryDir, 'ablation_2d.csv'); end

variants = {'APF-only', 'Informed-RRT*', 'w/o ellipse seeding', ...
    'w/o path-bias sampling', 'w/o dynamic rewiring', 'w/o seed shortcutting', 'AB-IRRT* (full)'};
scenes = {'一般', '狭窄', '广阔'};
modes = {'y', 'n', 'g'};

rows = struct('variant', {}, 'scenario', {}, 'run', {}, 'seed', {}, 'success', {}, ...
    'length', {}, 'time', {}, 'firstSolutionTime', {}, 'tPre', {}, 'tOpt', {}, ...
    'nodes', {}, 'turnSum', {}, 'minClear', {}, 'escapes', {}, 'sideFlips', {}, 'fallback', {});
z = 0;
baseSeed = 20260903;
for c = 1:numel(modes)
    env = bair_env2d(modes{c});
    qf = @(pts) query_pts(pts, env);
    for r = 1:runs
        seed = baseSeed + 1000 * c + r;
        for v = 1:numel(variants)
            rng(seed, 'twister');
            res = run_variant(variants{v}, env, opts);
            z = z + 1;
            m = metrics_of(res.path, qf);
            rows(z) = struct('variant', variants{v}, 'scenario', scenes{c}, 'run', r, ...
                'seed', seed, 'success', res.success, 'length', res.length, 'time', res.time, ...
                'firstSolutionTime', res.firstSol, 'tPre', res.tPre, 'tOpt', res.tOpt, ...
                'nodes', res.nodes, 'turnSum', m.turnSum, 'minClear', m.minClear, ...
                'escapes', res.escapes, 'sideFlips', res.flips, 'fallback', res.fb);
        end
        if ~opts.quiet && mod(r, max(1, floor(runs / 5))) == 0
            log_progress('  ablation %s: run %d/%d done', scenes{c}, r, runs);
        end
    end
    if ~opts.quiet, log_progress('ablation scene %s complete.', scenes{c}); end
end
results = struct2table(rows);
writetable(results, opts.outputFile, 'Encoding', 'UTF-8');
fprintf('2D ablation complete -> %s\n', opts.outputFile);
end

function res = run_variant(variant, env, opts)
res = struct('success', false, 'length', inf, 'time', NaN, 'firstSol', NaN, ...
    'tPre', NaN, 'tOpt', NaN, 'nodes', NaN, 'escapes', NaN, 'flips', NaN, 'fb', false, ...
    'path', zeros(0, 2));

preOpts = struct('useFallback', true, 'step', opts.step);
tp0 = tic; [seedPath, apfInfo] = stable_apf2d(env, preOpts); tPre = toc(tp0);
seed = seedPath; if ~apfInfo.success, seed = []; end
res.escapes = apfInfo.escapes; res.flips = apfInfo.sideFlips; res.fb = apfInfo.fallback;

% 种子捷径化（除“w/o seed shortcutting”变体外均启用，与主管线一致）
if ~isempty(seed) && ~strcmp(variant, 'w/o seed shortcutting')
    ts0 = tic;
    [seed, ~, ~, ~] = shortcut_seed2d(seed, env, 1.15);
    tPre = tPre + toc(ts0);
end

ro = struct('maxIter', opts.maxIter, 'step', opts.step, 'goalBias', opts.goalBias, ...
    'rewireRadius', opts.rewireRadius, 'pbias', 0.0, 'dynamicRewire', false, ...
    'rewireMax', opts.rewireMax, 'useSeedEllipse', true);

switch variant
    case 'APF-only'
        ao = struct('useFallback', false, 'step', opts.step);
        t0 = tic; [p, ai] = stable_apf2d(env, ao); tt = toc(t0);
        res.success = ai.success; res.path = p; res.length = ai.length;
        res.time = tt; res.tPre = tt; res.tOpt = 0;
        if ai.success, res.firstSol = ai.timeLocal; end
        res.escapes = ai.escapes; res.flips = ai.sideFlips; res.fb = ai.fallback;
        return;
    case 'Informed-RRT*'
        t0 = tic; [p, ri] = informed_rrtstar2d(env, [], ro); tt = toc(t0);
        res.tPre = 0;
    case 'w/o ellipse seeding'
        ro.pbias = opts.pbias; ro.useSeedEllipse = false;
        t0 = tic; [p, ri] = informed_rrtstar2d(env, seed, ro); tt = toc(t0);
        res.tPre = tPre;
    case 'w/o path-bias sampling'
        ro.pbias = 0; ro.useSeedEllipse = true;
        t0 = tic; [p, ri] = informed_rrtstar2d(env, seed, ro); tt = toc(t0);
        res.tPre = tPre;
    case 'w/o dynamic rewiring'
        ro.pbias = opts.pbias; ro.useSeedEllipse = true; ro.dynamicRewire = false;
        ro.rewireRadius = opts.rewireMax;
        t0 = tic; [p, ri] = informed_rrtstar2d(env, seed, ro); tt = toc(t0);
        res.tPre = tPre;
    case 'w/o seed shortcutting'
        ro.pbias = opts.pbias; ro.useSeedEllipse = true; ro.dynamicRewire = true;
        t0 = tic; [p, ri] = informed_rrtstar2d(env, seed, ro); tt = toc(t0);
        res.tPre = tPre;
    case 'AB-IRRT* (full)'
        ro.pbias = opts.pbias; ro.useSeedEllipse = true; ro.dynamicRewire = true;
        t0 = tic; [p, ri] = informed_rrtstar2d(env, seed, ro); tt = toc(t0);
        res.tPre = tPre;
    otherwise
        error('unknown ablation variant: %s', variant);
end

res.tOpt = tt;
res.time = res.tPre + tt;
res.path = p;
res.success = logical(ri.success) && size(p, 1) >= 2;
if res.success
    res.length = path_length(p);
    if isfinite(ri.firstSolutionTime)
        res.firstSol = res.tPre + ri.firstSolutionTime;
    elseif ~isempty(seed)
        res.firstSol = res.tPre;
    end
else
    res.length = inf;
end
res.nodes = ri.nodes;
end

function m = metrics_of(path, qf)
m = struct('turnSum', NaN, 'minClear', NaN);
if size(path, 1) >= 2
    pm = path_metrics_nd(path, qf);
    m.turnSum = pm.turnSum; m.minClear = pm.minClear;
end
end

function d = query_pts(pts, env)
d = zeros(size(pts, 1), 1);
for i = 1:size(pts, 1), d(i) = obstacle_query2d(pts(i, :), env); end
end

function L = path_length(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
