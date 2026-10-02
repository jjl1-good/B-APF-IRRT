function [path, info] = rrtconnect2d(env, opts)
%RRTCONNECT2D 双向 RRT-Connect 基线（快速可行解规划器）。
% 新增：首解时间与计时拆分字段（anytime 历史为单点：首解即终止）。
if nargin < 2, opts = struct; end
if ~isfield(opts, 'maxIter'), opts.maxIter = 4000; end
if ~isfield(opts, 'step'), opts.step = 5; end
if ~isfield(opts, 'goalBias'), opts.goalBias = 0.1; end
Xa = env.start; Pa = 0; Xb = env.goal; Pb = 0; found = false; ia = 0; ib = 0;
firstSolTime = NaN; it = 0;
t0 = tic;
if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end
tWall = tic;                      % 等时间对照用：墙钟上限（默认 Inf 表示不启用）
Sm = zeros(opts.maxIter, 2); nS = 0;   % 记录采样点（仅用于采样密集度图，不影响随机数消耗）
for it = 1:opts.maxIter
    if toc(tWall) > opts.timeLimit, break; end
    if rand < opts.goalBias, xr = env.goal; else, xr = [env.bounds(1) + (env.bounds(2) - env.bounds(1)) * rand, env.bounds(3) + (env.bounds(4) - env.bounds(3)) * rand]; end
    nS = nS + 1; Sm(nS, :) = xr(:)';
    [Xa, Pa, ia, ok] = extend(Xa, Pa, xr, env, opts.step);
    if ~ok, [Xa, Xb] = swap(Xa, Xb); [Pa, Pb] = swap(Pa, Pb); continue; end
    [Xb, Pb, ib, met] = connect(Xb, Pb, Xa(ia, :), env, opts.step);
    if met, found = true; firstSolTime = toc(t0); break; end
    [Xa, Xb] = swap(Xa, Xb); [Pa, Pb] = swap(Pa, Pb);
end
if found
    % 两棵树在迭代中会被交换，因此按“根节点是否为起点”判定方向后再拼接，
    % 保证返回路径始终为 start -> 汇合点 -> goal。
    if norm(Xa(1, :) - env.start(:)') < 1e-9
        pa = tracePath(Xa, Pa, ia);   % start -> 汇合点
        pb = tracePath(Xb, Pb, ib);   % goal  -> 汇合点
    else
        pa = tracePath(Xb, Pb, ib);
        pb = tracePath(Xa, Pa, ia);
    end
    raw = [pa; flipud(pb(1:end - 1, :))];
    pShort = raw; tShortcut = 0;   % 终路径贪心捷径后处理已取消（2026-09-20）
    % 短路后逐段复检（不消耗随机数）：异常时回退到未短路路径，再不行则判为失败。
    if segments_free(pShort, env)
        path = pShort;
    elseif segments_free(raw, env)
        path = raw; info.reverted = true;
    else
        path = zeros(0, 2); found = false; info.rawBlocked = true;
    end
else
    path = zeros(0, 2); tShortcut = 0;
end
tPlanner = toc(t0);
if found
    hist = [firstSolTime, path_length2d(path)];
else
    hist = zeros(0, 2);
end
info = struct('success', found, 'length', path_length2d(path), ...
    'nodes', size(Xa, 1) + size(Xb, 1), 'iterations', it, 'goalFound', found, ...
    'firstSolutionTime', firstSolTime, 'bestHistory', hist, ...
    'samples', Sm(1:nS, :), ...
    'tPlanner', tPlanner, 'tShortcut', tShortcut, 'timeTotal', toc(t0));
end
function [X, P, idx, ok] = extend(X, P, target, e, step)
[~, idx] = min(sum((X - target).^2, 2)); d = target - X(idx, :); ok = false;
if norm(d) < eps, return; end
q = X(idx, :) + min(step, norm(d)) * d / norm(d);
if ~inside(q, e.bounds) || ~collisionChecking(X(idx, :), q, e.squareAll, e.round), return; end
X(end + 1, :) = q; P(end + 1) = idx; idx = size(X, 1); ok = true;
end
function [X, P, idx, met] = connect(X, P, target, e, step)
met = false; idx = 0;
while true
    [X, P, idx, ok] = extend(X, P, target, e, step); if ~ok, return; end
    if norm(X(idx, :) - target) <= step * 1.01
        % 精确接入目标节点，保证两棵树重构路径端点一致；
        % 若末段被阻挡则不视为已连接，继续搜索。
        if norm(X(idx, :) - target) > 1e-9
            if ~collisionChecking(X(idx, :), target, e.squareAll, e.round), return; end
            X(end + 1, :) = target; P(end + 1) = idx; idx = size(X, 1);
        end
        met = true; return;
    end
end
end
function p = tracePath(X, P, idx)
p = X(idx, :); while idx > 1, idx = P(idx); p(end + 1, :) = X(idx, :); end; p = flipud(p);
end
function [a, b] = swap(a, b), c = a; a = b; b = c; end
function ok = inside(x, b), ok = x(1) >= b(1) && x(1) <= b(2) && x(2) >= b(3) && x(2) <= b(4); end
function p = shortcut(p, e)
if size(p, 1) < 3, return; end
i = 1;
while i < size(p, 1) - 1
    j = size(p, 1);
    while j > i + 1 && ~collisionChecking(p(i, :), p(j, :), e.squareAll, e.round), j = j - 1; end
    p = [p(1:i, :); p(j:end, :)]; %#ok<AGROW>
    i = i + 1;
end
end
function L = path_length2d(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end

function ok = segments_free(p, e)
ok = true;
for i = 1:size(p, 1) - 1
    if ~collisionChecking(p(i, :), p(i + 1, :), e.squareAll, e.round), ok = false; return; end
end
end
