function results = run_3d_comparison(runs, opts)
%RUN_3D_COMPARISON 三维统一对照实验（修订版）。
%
% 场景：general（原一般）、narrow（原狭窄）、suspended（悬空球阵）、
%       ring（环形柱廊）、overhang（悬挑平台）——后三个为新增真三维场景。
% 算法：RRT* / Informed-RRT* / RRT-Connect / PRM* / FMT* / BIT* / RRT# /
%       APF-only / AB-IRRT*（本文方法）。
%
% opts.scenes : 只运行指定场景（cellstr，mode 名），默认全部；
% opts.tag    : 结果文件后缀标签（分场景并行/分批运行用）。
%
% 用法：
%   R = run_3d_comparison(50);                       % 全部场景
%   R = run_3d_comparison(50, struct('scenes',{'ring'},'tag','ring'));

if nargin < 1 || isempty(runs), runs = 50; end
if nargin < 2, opts = struct; end
def = struct('maxIter', 6000, 'step', 6, 'goalBias', 0.08, 'rewireRadius', 18, ...
    'outputFile', '', 'anytimeFile', '', 'quiet', false, 'tag', '', ...
    'scenes', {{'general', 'narrow', 'suspended', 'ring', 'overhang'}});
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));

suffix = '';
if ~isempty(opts.tag), suffix = ['_', opts.tag]; end
if isempty(opts.outputFile)
    opts.outputFile = fullfile(entryDir, ['results_3d_v2', suffix, '.csv']);
end
if isempty(opts.anytimeFile)
    opts.anytimeFile = fullfile(entryDir, ['anytime_3d_v2', suffix, '.mat']);
end

modeList = opts.scenes;
sceneNames = containers.Map( ...
    {'general', 'general_v2', 'narrow', 'suspended', 'ring', 'overhang'}, ...
    {'一般', '一般', '狭窄', '悬空', '环廊', '悬挑'});
algs = {'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only', 'AB-IRRT*'};
if isfield(opts, 'algs') && ~isempty(opts.algs), algs = opts.algs; end
nAlg = numel(algs);

rows = struct('scenario', {}, 'algorithm', {}, 'run', {}, 'seed', {}, 'success', {}, ...
    'length', {}, 'time', {}, 'firstSolutionTime', {}, 'tPlanner', {}, 'tShortcut', {}, ...
    'tPre', {}, 'tOpt', {}, 'nodes', {}, 'iterations', {}, 'turnSum', {}, 'minClear', {}, ...
    'straightness', {}, 'escapes', {}, 'sideFlips', {}, 'fallback', {});
anytime = struct('scenario', {}, 'algorithm', {}, 'run', {}, 't', {}, 'cost', {});
z = 0; za = 0;
baseSeed = 20260903;

for c = 1:numel(modeList)
    mode = modeList{c};
    env = bair_env3d(mode);
    isFree = @(a, b) collisionChecking3D(a, b, env);
    qf = @(pts) query_pts3(pts, env);
    sName = sceneNames(mode);
    for r = 1:runs
        seed = baseSeed + 1000 * c + r;
        for a = 1:nAlg
            rng(seed, 'twister');
            [ok, L, tt, firstT, tP, tS, tPre, tOpt, nodes, iters, esc, flip, fb, path, hist] ...
                = run_one3(algs{a}, env, isFree, opts, mode);
            z = z + 1;
            m = metrics_of3(path, qf);
            rows(z) = struct('scenario', sName, 'algorithm', algs{a}, 'run', r, ...
                'seed', seed, 'success', ok, 'length', L, 'time', tt, ...
                'firstSolutionTime', firstT, 'tPlanner', tP, 'tShortcut', tS, ...
                'tPre', tPre, 'tOpt', tOpt, 'nodes', nodes, 'iterations', iters, ...
                'turnSum', m.turnSum, 'minClear', m.minClear, ...
                'straightness', m.straightness, 'escapes', esc, 'sideFlips', flip, ...
                'fallback', fb);
            if ~isempty(hist)
                za = za + 1;
                anytime(za) = struct('scenario', sName, 'algorithm', algs{a}, ...
                    'run', r, 't', hist(:, 1), 'cost', hist(:, 2));
            end
        end
        if ~opts.quiet && mod(r, max(1, floor(runs / 10))) == 0
            log_progress('  3D %s: run %d/%d done', sName, r, runs);
        end
    end
    if ~opts.quiet, log_progress('3D scene %s complete.', sName); end
end

T = struct2table(rows);
writetable(T, opts.outputFile, 'Encoding', 'UTF-8');
save(opts.anytimeFile, 'anytime', '-v7.3');
results = T;
fprintf('3D comparison complete: %d scenes x %d algorithms x %d runs -> %s\n', numel(modeList), nAlg, runs, opts.outputFile);
end

function [ok, L, tt, firstT, tP, tS, tPre, tOpt, nodes, iters, esc, flip, fb, path, hist] ...
    = run_one3(alg, env, isFree, opts, mode)
tPre = NaN; tOpt = NaN; esc = NaN; flip = NaN; fb = false; hist = zeros(0, 2);
common = rmfield(opts, intersect(fieldnames(opts), ...
    {'outputFile', 'anytimeFile', 'quiet', 'scenes', 'tag'}));
common = rmfield(common, intersect(fieldnames(common), {'maxNodes', 'batchSize', 'gamma', 'kMax'}));
switch alg
    case 'RRT*'
        t0 = tic; [path, info] = rrtstar3d(env, common); tt = toc(t0);
    case 'Informed-RRT*'
        t0 = tic; [path, info] = informed_rrtstar3d(env, [], common); tt = toc(t0);
    case 'RRT-Connect'
        t0 = tic; [path, info] = rrtconnect3d(env, common); tt = toc(t0);
    case 'PRM*'
        po = common; po.gamma = 1.0; po.kMax = 12; po.maxNodes = common.maxIter;
        t0 = tic; [path, info] = prmstar_nd(env.bounds, env.start, env.goal, isFree, po); tt = toc(t0);
    case 'FMT*'
        po = common; po.gamma = 0.7; po.maxNodes = common.maxIter;
        t0 = tic; [path, info] = fmtstar_nd(env.bounds, env.start, env.goal, isFree, po); tt = toc(t0);
    case 'BIT*'
        po = common; po.gamma = 0.65; po.maxNodes = common.maxIter;
        t0 = tic; [path, info] = bitstar_nd(env.bounds, env.start, env.goal, isFree, po); tt = toc(t0);
    case 'RRT#'
        t0 = tic; [path, info] = rrtsharp_nd(env.bounds, env.start, env.goal, isFree, common); tt = toc(t0);
    case 'APF-only'
        ao = common; ao.useFallback = false;
        t0 = tic; [path, info] = stable_apf3d(env, ao); tt = toc(t0);
        esc = info.escapes; flip = info.sideFlips; fb = info.fallback;
    case 'AB-IRRT*'
        bo = common; bo.pbias = 0.1; bo.dynamicRewire = true; bo.rewireMax = 30;
        bo.useFallback = true;
        t0 = tic; [~, ~, detail] = bair_core3d(mode, bo); tt = toc(t0);
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
L = path_length3(path);
if ~ok, L = inf; end
firstT = info.firstSolutionTime; tP = info.tPlanner; tS = info.tShortcut;
nodes = info.nodes; iters = info.iterations;
end

function m = metrics_of3(path, qf)
m = struct('turnSum', NaN, 'minClear', NaN, 'straightness', NaN);
if size(path, 1) >= 2
    pm = path_metrics_nd(path, qf);
    m.turnSum = pm.turnSum; m.minClear = pm.minClear; m.straightness = pm.straightness;
end
end

function d = query_pts3(pts, env)
d = zeros(size(pts, 1), 1);
for i = 1:size(pts, 1), d(i) = obstacle_query3d(pts(i, :), env); end
end

function L = path_length3(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
