function [L, T, detail] = bair_core3d(mode, opts)
%BAIR_CORE3D AB-IRRT* 三维完整管线（Bug-APF 预规划 + 增强 Informed-RRT* 优化）。
% 计时拆分与字段说明同 bair_core2d.m。

if nargin < 2, opts = struct; end
if ~isfield(opts, 'useFallback'), opts.useFallback = true; end
if ~isfield(opts, 'plot'), opts.plot = false; end
env = bair_env3d(mode);

% 可选：为“有足迹”的机器人把障碍膨胀 r（把机器人当质点）。
% 默认不启用（opts 里没有 inflate 时行为与主实验完全一致）。
if isfield(opts, 'inflate') && ~isempty(opts.inflate) && opts.inflate > 0
    env = inflate_env3d(env, opts.inflate);
end

% ---- 预规划阶段 ----
preOpts = opts; preOpts.useFallback = opts.useFallback;
tp0 = tic;
[guide, ai] = stable_apf3d(env, preOpts);
tPre = toc(tp0);

useLocal = true;
if isfield(opts, 'useLocal'), useLocal = opts.useLocal; end
useSeed = useLocal && ai.success && ~isempty(guide);

% ---- 种子捷径化（上界收紧，同二维；三维原有种子可达欧氏下界 2.5 倍）----
seedShortcut = true;
if isfield(opts, 'seedShortcut'), seedShortcut = opts.seedShortcut; end
ratioLimit = 1.15;
if isfield(opts, 'seedShortcutRatio'), ratioLimit = opts.seedShortcutRatio; end
seedInfo = struct('applied', false, 'lenRaw', inf, 'lenShort', inf);
if useSeed
    seedInfo.lenRaw = path_length3(guide);
    seedInfo.lenShort = seedInfo.lenRaw;
    if seedShortcut
        ts0 = tic;
        [guide, seedInfo.applied, seedInfo.lenRaw, seedInfo.lenShort] = ...
            shortcut_seed3d(guide, env, ratioLimit);
        tPre = tPre + toc(ts0);
    end
end

seed = guide; if ~useSeed, seed = []; end

% ---- 优化阶段 ----
if ~isfield(opts, 'useInformed'), opts.useInformed = true; end
if opts.useInformed
    if isfield(opts, 'timeLimit') && isfinite(opts.timeLimit)
        opts.timeLimit = max(1e-3, opts.timeLimit - tPre);   % 预规划与捷径化已经用掉的时间
    end
    to0 = tic;
    [path, ri] = informed_rrtstar3d(env, seed, opts);
    tOpt = toc(to0);
else
    path = seed; tOpt = 0;
    ri = struct('success', ~isempty(path), 'length', path_length3(path), ...
        'nodes', 0, 'iterations', 0, 'goalFound', false, 'seedUsed', useSeed, ...
        'firstSolutionTime', 0, 'bestHistory', zeros(0, 2), ...
        'tPlanner', 0, 'tShortcut', 0, 'timeTotal', 0);
end
if isempty(path), path = guide; end

L = path_length3(path); T = tPre + tOpt;
if useSeed
    firstSolutionTime = tPre;
else
    firstSolutionTime = tPre + ri.firstSolutionTime;
    if ~isfinite(firstSolutionTime), firstSolutionTime = NaN; end
end
hist = ri.bestHistory;
if ~isempty(hist), hist(:, 1) = hist(:, 1) + tPre; end

detail = struct('mode', mode, 'path', path, 'apfPath', guide, 'apf', ai, ...
    'rrt', ri, 'length', L, 'time', T, 'success', isfinite(L), ...
    'tPre', tPre, 'tOpt', tOpt, 'firstSolutionTime', firstSolutionTime, ...
    'bestHistory', hist, 'seedShortcut', seedInfo);

if opts.plot && ~isempty(path)
    figure; plot3(path(:, 1), path(:, 2), path(:, 3), 'r-', 'LineWidth', 2); grid on;
    axis equal; axis([env.bounds(1:2), env.bounds(3:4), env.bounds(5:6)]);
    title(['BAIR-3D ' mode]);
end
end

function L = path_length3(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
