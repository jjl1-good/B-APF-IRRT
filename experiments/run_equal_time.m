function run_equal_time(opts)
%RUN_EQUAL_TIME 等时间对照实验：所有算法共用同一个墙钟时间上限，再比较路径质量。
%
% 时间上限：以主实验中本文方法在该环境的实测平均总耗时为 1×（记作 T1），另取 2× 作为 T2。
% 运行设置：把迭代/样本预算放大到 1e5，使约束变成"时间"而不是"预算"；每个环境 30 次配对种子。
% 输出：equal_time_results.csv（逐次）、equal_time_summary.csv（按 环境×档位×算法 汇总）
% 可选参数：run_equal_time(struct('runs',30,'caps',[1 2],'envFilter',{'广阔'}))
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir); addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));

runs = 30;
baseSeed = 20260903;
caps = [1, 2];
maxIter = 1e5;
if nargin < 1, opts = struct; end
if isfield(opts, 'runs'), runs = opts.runs; end
if isfield(opts, 'caps'), caps = opts.caps; end

% ---- 每个环境的参考时间（本文方法实测平均总耗时） ----
envs = struct('dim', {}, 'mode', {}, 'name', {}, 'tRef', {}, 'c', {});
S2 = readtable(fullfile(entryDir, 'summary_2d.csv'), 'Encoding', 'UTF-8');
sc2 = {'一般', '狭窄', '广阔'};
for i = 1:numel(sc2)
    name = sc2{i};
    m = strcmp(S2.scenario, name) & strcmp(S2.algorithm, 'AB-IRRT*');
    envs(end + 1) = struct('dim', '2d', 'mode', mode_of(name), 'name', name, ...
        'tRef', S2.meanTime(find(m, 1)), 'c', i); %#ok<AGROW>
end
pairs = {'general_v2', 'general', '一般'; 'narrow', 'narrow', '狭窄'; ...
    'suspended', 'suspended', '悬空'; 'ring', 'ring', '环廊'; 'overhang', 'overhang', '悬挑'};
for i = 1:size(pairs, 1)
    S3 = readtable(fullfile(entryDir, sprintf('summary_3d_%s.csv', pairs{i, 2})), 'Encoding', 'UTF-8');
    m = strcmp(S3.algorithm, 'AB-IRRT*');
    envs(end + 1) = struct('dim', '3d', 'mode', pairs{i, 1}, 'name', pairs{i, 3}, ...
        'tRef', S3.meanTime(find(m, 1)), 'c', i); %#ok<AGROW>
end

algs = {'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only', 'AB-IRRT*'};
if isfield(opts, 'modeFilter') && ~isempty(opts.modeFilter)
    keep = ismember({envs.mode}, opts.modeFilter);
    envs = envs(keep);
end
if isfield(opts, 'envFilter') && ~isempty(opts.envFilter)
    keep = ismember({envs.name}, opts.envFilter);
    envs = envs(keep);
end
if isfield(opts, 'algs') && ~isempty(opts.algs), algs = opts.algs; end
rows = struct('dim', {}, 'scenario', {}, 'capTag', {}, 'capSeconds', {}, 'algorithm', {}, ...
    'run', {}, 'seed', {}, 'success', {}, 'length', {}, 'time', {}, 'firstSolutionTime', {});
z = 0;
tAll = tic;
for e = 1:numel(envs)
    E = envs(e);
    if strcmp(E.dim, '2d')
        env = bair_env2d(E.mode);
        isFree = @(a, b) collisionChecking(a, b, env.squareAll, env.round);
    else
        env = bair_env3d(E.mode);
        isFree = @(a, b) collisionChecking3D(a, b, env);
    end
    for ci = 1:numel(caps)
        cap = caps(ci) * E.tRef;
        for r = 1:runs
            seed = baseSeed + 1000 * E.c + r;
            for a = 1:numel(algs)
                opts = struct('maxIter', maxIter, 'maxNodes', maxIter, 'timeLimit', cap, ...
                    'step', 4, 'goalBias', 0.08, 'rewireRadius', 14, 'quiet', true);
                if strcmp(E.dim, '3d')
                    opts.step = 6; opts.rewireRadius = 18;
                end
                rng(seed, 'twister');
                try
                    [ok, L, tt, firstT] = run_one(algs{a}, env, isFree, opts, E);
                catch err
                    log_progress('  [err] %s %s cap%d run%d: %s', E.name, algs{a}, ci, r, err.message);
                    ok = false; L = inf; tt = NaN; firstT = NaN;
                end
                z = z + 1;
                rows(z) = struct('dim', E.dim, 'scenario', E.name, 'capTag', caps(ci), ...
                    'capSeconds', cap, 'algorithm', algs{a}, 'run', r, 'seed', seed, ...
                    'success', ok, 'length', L, 'time', tt, 'firstSolutionTime', firstT);
            end
        end
        log_progress('equal-time: %s cap %gx (%.3f s) done', E.name, caps(ci), cap);
    end
end

T = struct2table(rows);
writetable(T, fullfile(entryDir, 'equal_time_results.csv'), 'Encoding', 'UTF-8');

% ---- 汇总 ----
dm = unique(T.dim, 'stable'); sc = unique(T.scenario, 'stable');
capsU = unique(T.capTag, 'stable');
S = {};
for i = 1:numel(dm)
    for j = 1:numel(sc)
        sel0 = strcmp(T.dim, dm{i}) & strcmp(T.scenario, sc{j});
        if ~any(sel0), continue; end
        for k = 1:numel(capsU)
            for a = 1:numel(algs)
                m = sel0 & T.capTag == capsU(k) & strcmp(T.algorithm, algs{a});
                if ~any(m), continue; end
                g = T(m, :);
                good = g.success == 1;
                L = g.length(good);
                S(end + 1, :) = {dm{i}, sc{j}, capsU(k), g.capSeconds(1), algs{a}, ...
                    height(g), 100 * mean(g.success), mean_or_nan(L), std_or_nan(L), ...
                    mean_or_nan(g.time), mean_or_nan(g.firstSolutionTime)}; %#ok<AGROW>
            end
        end
    end
end
Ts = cell2table(S, 'VariableNames', {'dim', 'scenario', 'capTag', 'capSeconds', 'algorithm', ...
    'runs', 'successRate', 'meanLength', 'stdLength', 'meanTime', 'meanFirstSol'});
writetable(Ts, fullfile(entryDir, 'equal_time_summary.csv'), 'Encoding', 'UTF-8');
fprintf('equal-time experiment done in %.1f min -> equal_time_summary.csv\n', toc(tAll) / 60);
end

function [ok, L, tt, firstT] = run_one(alg, env, isFree, opts, E)
common = opts;
switch alg
    case 'RRT*'
        if strcmp(E.dim, '2d'), t0 = tic; [path, info] = rrtstar2d(env, common);
        else, t0 = tic; [path, info] = rrtstar3d(env, common); end
        tt = toc(t0);
    case 'Informed-RRT*'
        if strcmp(E.dim, '2d'), t0 = tic; [path, info] = informed_rrtstar2d(env, [], common);
        else, t0 = tic; [path, info] = informed_rrtstar3d(env, [], common); end
        tt = toc(t0);
    case 'RRT-Connect'
        if strcmp(E.dim, '2d'), t0 = tic; [path, info] = rrtconnect2d(env, common);
        else, t0 = tic; [path, info] = rrtconnect3d(env, common); end
        tt = toc(t0);
    case 'PRM*'
        po = common; po.gamma = 1.0; po.kMax = 12;
        if strcmp(E.dim, '3d'), po.gamma = 1.0; end
        t0 = tic; [path, info] = prmstar_nd(env.bounds, env.start, env.goal, isFree, po); tt = toc(t0);
    case 'FMT*'
        po = common; po.gamma = 0.8;
        if strcmp(E.dim, '3d'), po.gamma = 0.7; end
        t0 = tic; [path, info] = fmtstar_nd(env.bounds, env.start, env.goal, isFree, po); tt = toc(t0);
    case 'BIT*'
        po = common; po.gamma = 0.9;
        if strcmp(E.dim, '3d'), po.gamma = 0.65; end
        t0 = tic; [path, info] = bitstar_nd(env.bounds, env.start, env.goal, isFree, po); tt = toc(t0);
    case 'RRT#'
        t0 = tic; [path, info] = rrtsharp_nd(env.bounds, env.start, env.goal, isFree, common); tt = toc(t0);
    case 'APF-only'
        ao = common; ao.useFallback = false;
        if strcmp(E.dim, '2d'), t0 = tic; [path, info] = stable_apf2d(env, ao);
        else, t0 = tic; [path, info] = stable_apf3d(env, ao); end
        tt = toc(t0);
    case 'AB-IRRT*'
        bo = common;
        if strcmp(E.dim, '2d'), bo.pbias = 0.5; bo.rewireMax = 16; else, bo.pbias = 0.1; bo.rewireMax = 30; end
        bo.dynamicRewire = true; bo.useFallback = true;
        t0 = tic; [~, ~, detail] = bair_core2d_or_3d(E.dim, E.mode, bo); tt = toc(t0);
        path = detail.path; info = detail.rrt; info.firstSolutionTime = detail.firstSolutionTime;
    otherwise
        error('unknown algorithm %s', alg);
end
if ~isfield(info, 'firstSolutionTime') || isempty(info.firstSolutionTime), info.firstSolutionTime = NaN; end
if size(path, 1) >= 2 && isnan(info.firstSolutionTime), info.firstSolutionTime = tt; end
ok = logical(info.success) && size(path, 1) >= 2;
if ok
    % 与 path_length / path_length3 等价的通用折线长度（维度无关）
    L = sum(sqrt(sum(diff(path) .^ 2, 2)));
else
    L = inf;
end
firstT = info.firstSolutionTime;
end

function [a, t, detail] = bair_core2d_or_3d(dim, mode, opts)
if strcmp(dim, '2d')
    [a, t, detail] = bair_core2d(mode, opts);
else
    [a, t, detail] = bair_core3d(mode, opts);
end
end

function m = mode_of(name)
switch name
    case '一般', m = 'y';
    case '狭窄', m = 'n';
    otherwise, m = 'g';
end
end

function v = mean_or_nan(x)
v = mean(x(~isnan(x)));
if isempty(v), v = NaN; end
end

function v = std_or_nan(x)
x = x(~isnan(x));
if numel(x) > 1, v = std(x); else, v = NaN; end
end
