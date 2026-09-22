function results = run_2d_comparison(runs, opts)
%RUN_2D_COMPARISON 二维统一对照实验（修订版，8 种算法 × 3 个场景 × runs 次）。
%
% 算法（同参数预算、同碰撞检测、同种子序列）：
%   RRT* / Informed-RRT* / RRT-Connect / PRM* / FMT* / BIT* / RRT# /
%   APF-only / AB-IRRT*（本文方法，含 Bug-APF 预规划与增强优化）
%
% 指标（每场景×算法×运行一行）：
%   success, length, time（总）, firstSolutionTime, tPlanner, tShortcut,
%   tPre/tOpt（仅 AB-IRRT*）, nodes, iterations, turnSum, minClear,
%   straightness, escapes, sideFlips, fallback
%
% 配对设计：同一 (场景, 运行号) 下所有算法共享同一随机种子，
% 支持配对置换检验/效应量分析（审稿意见 R3-4）。
%
% anytime 历史（时间-最优代价曲线）保存为 .mat 结构数组。
%
% 用法：
%   R = run_2d_comparison(50);                     % 默认 50 次重复
%   R = run_2d_comparison(50, struct('maxIter',2000));

if nargin < 1 || isempty(runs), runs = 50; end
if nargin < 2, opts = struct; end
def = struct('maxIter', 2000, 'step', 4, 'goalBias', 0.08, 'rewireRadius', 14, ...
    'apfStep', 2, 'safetyMargin', 0, 'outputFile', '', 'anytimeFile', '', ...
    'quiet', false);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));

if isempty(opts.outputFile), opts.outputFile = fullfile(entryDir, 'results_2d_v2.csv'); end
if isempty(opts.anytimeFile), opts.anytimeFile = fullfile(entryDir, 'anytime_2d_v2.mat'); end

scenes = {'一般', '狭窄', '广阔'};
modes = {'y', 'n', 'g'};
algs = {'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only', 'AB-IRRT*'};
if isfield(opts, 'algs') && ~isempty(opts.algs), algs = opts.algs; end
nAlg = numel(algs);
nSc = numel(scenes);

rows = struct('scenario', {}, 'algorithm', {}, 'run', {}, 'seed', {}, 'success', {}, ...
    'length', {}, 'time', {}, 'firstSolutionTime', {}, 'tPlanner', {}, 'tShortcut', {}, ...
    'tPre', {}, 'tOpt', {}, 'nodes', {}, 'iterations', {}, 'turnSum', {}, 'minClear', {}, ...
    'straightness', {}, 'escapes', {}, 'sideFlips', {}, 'fallback', {});
anytime = struct('scenario', {}, 'algorithm', {}, 'run', {}, 't', {}, 'cost', {});
z = 0; za = 0;
baseSeed = 20260903;

for c = 1:nSc
    env = bair_env2d(modes{c});
    isFree = @(a, b) collisionChecking(a, b, env.squareAll, env.round);
    qf = @(pts) query_pts(pts, env);
    for r = 1:runs
        seed = baseSeed + 1000 * c + r;
        for a = 1:nAlg
            rng(seed, 'twister');
            tAlg = tic;
            [ok, L, tt, firstT, tP, tS, tPre, tOpt, nodes, iters, esc, flip, fb, path, hist] ...
                = run_one(algs{a}, env, isFree, opts, mode_idx(modes{c}));
            tsAlg = toc(tAlg);
            if tsAlg > 5
                log_progress('  [slow] 2D %s %s run %d: %.1f s', scenes{c}, algs{a}, r, tsAlg);
            end
            z = z + 1;
            m = metrics_of(path, qf);
            rows(z) = struct('scenario', scenes{c}, 'algorithm', algs{a}, 'run', r, ...
                'seed', seed, 'success', ok, 'length', L, 'time', tt, ...
                'firstSolutionTime', firstT, 'tPlanner', tP, 'tShortcut', tS, ...
                'tPre', tPre, 'tOpt', tOpt, 'nodes', nodes, 'iterations', iters, ...
                'turnSum', m.turnSum, 'minClear', m.minClear, ...
                'straightness', m.straightness, 'escapes', esc, 'sideFlips', flip, ...
                'fallback', fb);
            if ~isempty(hist)
                za = za + 1;
                anytime(za) = struct('scenario', scenes{c}, 'algorithm', algs{a}, ...
                    'run', r, 't', hist(:, 1), 'cost', hist(:, 2));
            end
        end
        if ~opts.quiet && mod(r, max(1, floor(runs / 10))) == 0
            log_progress('  2D %s: run %d/%d done', scenes{c}, r, runs);
        end
    end
    if ~opts.quiet, log_progress('2D scene %s complete.', scenes{c}); end
end

T = struct2table(rows);
writetable(T, opts.outputFile, 'Encoding', 'UTF-8');
save(opts.anytimeFile, 'anytime', '-v7.3');
results = T;
fprintf('2D comparison complete: %d scenes x %d algorithms x %d runs -> %s\n', nSc, nAlg, runs, opts.outputFile);
end

function ci = mode_idx(mode)
switch mode
    case 'y', ci = 'y';
    case 'n', ci = 'n';
    case 'g', ci = 'g';
    otherwise, ci = mode;
end
end

function [ok, L, tt, firstT, tP, tS, tPre, tOpt, nodes, iters, esc, flip, fb, path, hist] ...
    = run_one(alg, env, isFree, opts, mode)
% 统一执行单个算法一次，输出标量指标与路径/历史。
tPre = NaN; tOpt = NaN; esc = NaN; flip = NaN; fb = false; hist = zeros(0, 2);
common = rmfield(opts, intersect(fieldnames(opts), {'outputFile', 'anytimeFile', 'quiet', 'apfStep'}));
switch alg
    case 'RRT*'
        t0 = tic; [path, info] = rrtstar2d(env, common); tt = toc(t0);
    case 'Informed-RRT*'
        t0 = tic; [path, info] = informed_rrtstar2d(env, [], common); tt = toc(t0);
    case 'RRT-Connect'
        t0 = tic; [path, info] = rrtconnect2d(env, common); tt = toc(t0);
    case 'PRM*'
        po = common; po.maxNodes = common.maxIter; po.gamma = 1.0; po.kMax = 12;
        t0 = tic; [path, info] = prmstar_nd(env.bounds, env.start, env.goal, isFree, po); tt = toc(t0);
    case 'FMT*'
        po = common; po.maxNodes = common.maxIter; po.gamma = 0.8;
        t0 = tic; [path, info] = fmtstar_nd(env.bounds, env.start, env.goal, isFree, po); tt = toc(t0);
    case 'BIT*'
        po = common; po.maxNodes = common.maxIter; po.gamma = 0.9;
        t0 = tic; [path, info] = bitstar_nd(env.bounds, env.start, env.goal, isFree, po); tt = toc(t0);
    case 'RRT#'
        t0 = tic; [path, info] = rrtsharp_nd(env.bounds, env.start, env.goal, isFree, common); tt = toc(t0);
    case 'APF-only'
        ao = common; ao.useFallback = false;
        t0 = tic; [path, info] = stable_apf2d(env, ao); tt = toc(t0);
        esc = info.escapes; flip = info.sideFlips; fb = info.fallback;
    case 'AB-IRRT*'
        bo = common; bo.pbias = 0.5; bo.dynamicRewire = true; bo.rewireMax = 16;
        t0 = tic; [~, ~, detail] = bair_core2d(mode, bo); tt = toc(t0);
        path = detail.path; info = detail.rrt;
        tPre = detail.tPre; tOpt = detail.tOpt; hist = detail.bestHistory;
        info.firstSolutionTime = detail.firstSolutionTime;
        esc = detail.apf.escapes; flip = detail.apf.sideFlips; fb = detail.apf.fallback;
    otherwise
        error('unknown algorithm %s', alg);
end

if ~isfield(info, 'firstSolutionTime') || isempty(info.firstSolutionTime), info.firstSolutionTime = NaN; end
if ~isfield(info, 'tPlanner') || isempty(info.tPlanner), info.tPlanner = tt; end
if ~isfield(info, 'tShortcut') || isempty(info.tShortcut), info.tShortcut = NaN; end
if ~isfield(info, 'nodes'), info.nodes = NaN; end
if ~isfield(info, 'iterations'), info.iterations = NaN; end
if size(hist, 1) == 0 && isfield(info, 'bestHistory'), hist = info.bestHistory; end
if size(path, 1) >= 2 && isnan(info.firstSolutionTime), info.firstSolutionTime = tt; end

ok = logical(info.success) && size(path, 1) >= 2;
L = path_length(path);
if ~ok, L = inf; end
firstT = info.firstSolutionTime; tP = info.tPlanner; tS = info.tShortcut;
nodes = info.nodes; iters = info.iterations;
end

function m = metrics_of(path, qf)
m = struct('turnSum', NaN, 'minClear', NaN, 'straightness', NaN);
if size(path, 1) >= 2
    pm = path_metrics_nd(path, qf);
    m.turnSum = pm.turnSum; m.minClear = pm.minClear; m.straightness = pm.straightness;
end
end

function d = query_pts(pts, env)
d = zeros(size(pts, 1), 1);
for i = 1:size(pts, 1), d(i) = obstacle_query2d(pts(i, :), env); end
end

function L = path_length(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
