function [path, info] = rrtconnect3d(env, opts)
%RRTCONNECT3D 双向 RRT-Connect 三维基线（快速可行解规划器）。
% 新增：首解时间与计时拆分字段。
if nargin < 2, opts = struct; end
if ~isfield(opts, 'maxIter'), opts.maxIter = 4000; end
if ~isfield(opts, 'step'), opts.step = 7; end
Xa = env.start; Pa = 0; Xb = env.goal; Pb = 0; found = false; ia = 0; ib = 0;
firstSolTime = NaN; it = 0;
t0 = tic;
if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end
tWall = tic;                      % 等时间对照用：墙钟上限（默认 Inf 表示不启用）
Sm = zeros(opts.maxIter, 3); nS = 0;   % 记录采样点（仅用于采样密集度图，不影响随机数消耗）
for it = 1:opts.maxIter
    if toc(tWall) > opts.timeLimit, break; end
    xr = [randb(env.bounds(1), env.bounds(2)), randb(env.bounds(3), env.bounds(4)), randb(env.bounds(5), env.bounds(6))];
    nS = nS + 1; Sm(nS, :) = xr(:)';
    [Xa, Pa, ia, ok] = extend3(Xa, Pa, xr, env, opts.step);
    if ~ok, [Xa, Xb] = swap3(Xa, Xb); [Pa, Pb] = swap3(Pa, Pb); continue; end
    [Xb, Pb, ib, met] = connect3(Xb, Pb, Xa(ia, :), env, opts.step);
    if met, found = true; firstSolTime = toc(t0); break; end
    [Xa, Xb] = swap3(Xa, Xb); [Pa, Pb] = swap3(Pa, Pb);
end
if found
    % 两棵树在迭代中会被交换：按根节点判定方向，保证为 start -> 汇合点 -> goal。
    if norm(Xa(1, :) - env.start(:)') < 1e-9
        pa = trace3(Xa, Pa, ia);      % start -> 汇合点
        pb = trace3(Xb, Pb, ib);      % goal  -> 汇合点
    else
        pa = trace3(Xb, Pb, ib);
        pb = trace3(Xa, Pa, ia);
    end
    raw = [pa; flipud(pb(1:end - 1, :))];
    pShort = raw; tShortcut = 0;   % 终路径贪心捷径后处理已取消（2026-09-20）
    % 短路后逐段复检（不消耗随机数）：异常时回退到未短路路径，再不行判为失败。
    if segments_free3(pShort, env)
        path = pShort;
    elseif segments_free3(raw, env)
        path = raw; info.reverted = true;
    else
        path = zeros(0, 3); found = false; info.rawBlocked = true;
    end
else
    path = zeros(0, 3); tShortcut = 0;
end
tPlanner = toc(t0);
if found
    hist = [firstSolTime, path_length3(path)];
else
    hist = zeros(0, 2);
end
info = struct('success', found, 'length', path_length3(path), ...
    'nodes', size(Xa, 1) + size(Xb, 1), 'iterations', it, 'goalFound', found, ...
    'firstSolutionTime', firstSolTime, 'bestHistory', hist, ...
    'samples', Sm(1:nS, :), ...
    'tPlanner', tPlanner, 'tShortcut', tShortcut, 'timeTotal', toc(t0));
end
function [X, P, idx, ok] = extend3(X, P, target, e, step)
[~, idx] = min(sum((X - target).^2, 2)); v = target - X(idx, :); ok = false;
if norm(v) < eps, return; end
q = X(idx, :) + min(step, norm(v)) * v / norm(v);
if ~inside3(q, e.bounds) || ~collisionChecking3D(X(idx, :), q, e), return; end
X(end + 1, :) = q; P(end + 1) = idx; idx = size(X, 1); ok = true;
end
function [X, P, idx, met] = connect3(X, P, target, e, step)
met = false; idx = 0;
while true
    [X, P, idx, ok] = extend3(X, P, target, e, step); if ~ok, return; end
    if norm(X(idx, :) - target) <= step * 1.01
        % 精确接入目标节点；若末段被阻挡则不视为已连接，继续搜索。
        if norm(X(idx, :) - target) > 1e-9
            if ~collisionChecking3D(X(idx, :), target, e), return; end
            X(end + 1, :) = target; P(end + 1) = idx; idx = size(X, 1);
        end
        met = true; return;
    end
end
end
function p = trace3(X, P, idx)
p = X(idx, :); while idx > 1, idx = P(idx); p(end + 1, :) = X(idx, :); end; p = flipud(p);
end
function [a, b] = swap3(a, b), c = a; a = b; b = c; end
function x = randb(a, b), x = a + (b - a) * rand; end
function ok = inside3(p, b)
ok = p(1) >= b(1) && p(1) <= b(2) && p(2) >= b(3) && p(2) <= b(4) && ...
     p(3) >= b(5) && p(3) <= b(6);
end
function p = shortcut3(p, e)
if size(p, 1) < 3, return; end
i = 1;
while i < size(p, 1) - 1
    j = size(p, 1);
    while j > i + 1 && ~collisionChecking3D(p(i, :), p(j, :), e), j = j - 1; end
    p = [p(1:i, :); p(j:end, :)]; %#ok<AGROW>
    i = i + 1;
end
end
function L = path_length3(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end

function ok = segments_free3(p, e)
ok = true;
for i = 1:size(p, 1) - 1
    if ~collisionChecking3D(p(i, :), p(i + 1, :), e), ok = false; return; end
end
end
