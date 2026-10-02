function [a, t, detail] = bair_core2d(mode, opts)
%BAIR_CORE2D AB-IRRT* 二维完整管线（Bug-APF 预规划 + 增强 Informed-RRT* 优化）。
%
% 计时拆分（供审稿意见 R1-3 / R2-4 的“时间构成”统计）：
%   tPre  : 预规划阶段（局部规划器）
%   tOpt  : 优化阶段（Informed-RRT*）
%   firstSolutionTime : 首个可行路径可用时刻（预规划成功即为 tPre）
% detail.bestHistory 为整体 anytime 历史（已将优化阶段时间平移 tPre）。
%
% 消融选项：opts.useLocal=false 时退回“无预规划”（IRRT*-only 对照）；
%           opts.useInformed=false 时仅用预规划路径（APF-only 对照）。

if nargin < 2, opts = struct; end
env = bair_env2d(mode);
if ~isfield(opts, 'plot'), opts.plot = false; end

% ---- 预规划阶段 ----
preOpts = opts;
if ~isfield(preOpts, 'useFallback'), preOpts.useFallback = true; end
tp0 = tic;
[apfPath, apfInfo] = stable_apf2d(env, preOpts);
tPre = toc(tp0);

useLocal = true;
if isfield(opts, 'useLocal'), useLocal = opts.useLocal; end
useSeed = useLocal && apfInfo.success && ~isempty(apfPath);

% ---- 种子捷径化（上界收紧）----
% 预规划折线常有无谓绕行（二维广阔环境达欧氏下界 1.50 倍）；该路径同时充当
% c_best、椭圆初始大小与采样偏置目标，因此先用确定性贪心捷径化收紧上界。
% opts.seedShortcut = false 可关闭（消融用）；opts.seedShortcutRatio 默认 1.15。
seedShortcut = true;
if isfield(opts, 'seedShortcut'), seedShortcut = opts.seedShortcut; end
ratioLimit = 1.15;
if isfield(opts, 'seedShortcutRatio'), ratioLimit = opts.seedShortcutRatio; end
seedInfo = struct('applied', false, 'lenRaw', inf, 'lenShort', inf);
if useSeed
    seedInfo.lenRaw = path_length2d(apfPath);
    seedInfo.lenShort = seedInfo.lenRaw;
    if seedShortcut
        ts0 = tic;
        [apfPath, seedInfo.applied, seedInfo.lenRaw, seedInfo.lenShort] = ...
            shortcut_seed2d(apfPath, env, ratioLimit);
        tPre = tPre + toc(ts0);        % 捷径化耗时计入预规划（不藏时间）
    end
end

seed = apfPath; if ~useSeed, seed = []; end

% ---- 优化阶段 ----
if ~isfield(opts, 'useInformed'), opts.useInformed = true; end
if opts.useInformed
    if isfield(opts, 'timeLimit') && isfinite(opts.timeLimit)
        opts.timeLimit = max(1e-3, opts.timeLimit - tPre);   % 预规划与捷径化已经用掉的时间
    end
    to0 = tic;
    [path, rrtInfo] = informed_rrtstar2d(env, seed, opts);
    tOpt = toc(to0);
else
    path = seed; tOpt = 0;
    rrtInfo = struct('success', ~isempty(path), 'length', path_length2d(path), ...
        'nodes', 0, 'iterations', 0, 'goalFound', false, 'seedUsed', useSeed, ...
        'firstSolutionTime', 0, 'bestHistory', zeros(0, 2), ...
        'tPlanner', 0, 'tShortcut', 0, 'timeTotal', 0);
end
if isempty(path), path = apfPath; end

a = path_length2d(path); t = tPre + tOpt;

% ---- 首解时间与 anytime 历史（时间轴平移） ----
if useSeed
    firstSolutionTime = tPre;
else
    firstSolutionTime = tPre + rrtInfo.firstSolutionTime;
    if ~isfinite(firstSolutionTime), firstSolutionTime = NaN; end
end
hist = rrtInfo.bestHistory;
if ~isempty(hist), hist(:, 1) = hist(:, 1) + tPre; end

detail = struct('mode', mode, 'path', path, 'apfPath', apfPath, 'apf', apfInfo, ...
    'rrt', rrtInfo, 'length', a, 'time', t, 'success', isfinite(a), ...
    'tPre', tPre, 'tOpt', tOpt, 'firstSolutionTime', firstSolutionTime, ...
    'bestHistory', hist, 'seedShortcut', seedInfo);

if opts.plot
    figure('Name', ['BAIR ' upper(mode)]); hold on; axis equal;
    axis([env.bounds(1:2) env.bounds(3:4)]); box on;
    for k = 1:size(env.square, 1)
        rectangle('Position', env.square(k, :), 'FaceColor', [.15 .15 .15], 'EdgeColor', 'none');
    end
    for k = 1:size(env.round, 1)
        rectangle('Position', [env.round(k, 1) - env.round(k, 3), env.round(k, 2) - env.round(k, 3), 2 * env.round(k, 3), 2 * env.round(k, 3)], 'Curvature', [1 1], 'FaceColor', [.15 .15 .15], 'EdgeColor', 'none');
    end
    plot(env.start(1), env.start(2), 'mo', 'MarkerFaceColor', 'm');
    plot(env.goal(1), env.goal(2), 'go', 'MarkerFaceColor', 'g');
    if ~isempty(apfPath), plot(apfPath(:, 1), apfPath(:, 2), 'm--', 'LineWidth', 1); end
    if ~isempty(path), plot(path(:, 1), path(:, 2), 'r-', 'LineWidth', 2); end
    legend('APF guide', 'BAIR path', 'Location', 'best');
    title(['BAIR-' upper(mode)]);
end
end

function L = path_length2d(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
