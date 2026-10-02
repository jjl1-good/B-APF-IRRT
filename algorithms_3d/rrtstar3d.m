function [path, info] = rrtstar3d(env, opts)
%RRTSTAR3D 标准 RRT* 三维基线（均匀采样，固定重布线半径）。
% 新增：anytime 历史 (t, cbest) 与计时拆分。
if nargin < 2, opts = struct; end
if ~isfield(opts, 'maxIter'), opts.maxIter = 6000; end
if ~isfield(opts, 'step'), opts.step = 6; end
if ~isfield(opts, 'goalBias'), opts.goalBias = 0.08; end
if ~isfield(opts, 'rewireRadius'), opts.rewireRadius = 18; end
N = opts.maxIter + 2; X = zeros(N, 3); P = zeros(N, 1); C = inf(N, 1);
X(1, :) = env.start; C(1) = 0; n = 1; goalIdx = 0; best = inf;
hist = zeros(4096, 2); nh = 0; firstSolTime = NaN;
t0 = tic;
if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end
tWall = tic;                      % 等时间对照用：墙钟上限（默认 Inf 表示不启用）
for it = 1:opts.maxIter
    if toc(tWall) > opts.timeLimit, break; end
    if rand < opts.goalBias
        xr = env.goal;
    else
        xr = [randb(env.bounds(1), env.bounds(2)), randb(env.bounds(3), env.bounds(4)), ...
              randb(env.bounds(5), env.bounds(6))];
    end
    [~, near] = min(sum((X(1:n, :) - xr).^2, 2)); v = xr - X(near, :);
    if norm(v) < eps, continue; end
    xn = X(near, :) + min(opts.step, norm(v)) * v / norm(v);
    if ~inside3(xn, env.bounds) || ~collisionChecking3D(X(near, :), xn, env), continue; end
    dist = sqrt(sum((X(1:n, :) - xn).^2, 2)); cand = find(dist <= opts.rewireRadius);
    par = near; cc = C(near) + norm(xn - X(near, :));
    [~, ord] = sort(C(cand) + dist(cand)); cand = cand(ord);
    for z = 1:numel(cand)
        j = cand(z);
        if C(j) + dist(j) < cc - 1e-12 && collisionChecking3D(X(j, :), xn, env)
            par = j; cc = C(j) + dist(j);
        end
    end
    n = n + 1; X(n, :) = xn; P(n) = par; C(n) = cc;
    for z = 1:numel(cand)
        j = cand(z); nc = cc + norm(X(j, :) - xn);
        if j ~= par && nc < C(j) - 1e-12 && collisionChecking3D(xn, X(j, :), env)
            P(j) = n; C(j) = nc;
        end
    end
    dg = norm(xn - env.goal);
    if dg <= opts.step * 1.5 && collisionChecking3D(xn, env.goal, env)
        gc = cc + dg;
        if gc < best
            best = gc; goalIdx = n;
            if isnan(firstSolTime), firstSolTime = toc(t0); end
            nh = nh + 1; hist(nh, :) = [toc(t0), best];
        end
    end
end
tPlanner = toc(t0);
if goalIdx > 0
    q = goalIdx; path = env.goal;
    while q > 1, path(end + 1, :) = X(q, :); q = P(q); end %#ok<AGROW>
    path(end + 1, :) = env.start; path = flipud(path);
    tShortcut = 0;   % 终路径贪心捷径后处理已取消（2026-09-20）
else
    path = zeros(0, 3); tShortcut = 0;
end
hist = hist(1:nh, :);
info = struct('success', goalIdx > 0, 'length', path_length3(path), ...
    'nodes', n, 'iterations', opts.maxIter, 'goalFound', goalIdx > 0, ...
    'firstSolutionTime', firstSolTime, 'bestHistory', hist, ...
    'tPlanner', tPlanner, 'tShortcut', tShortcut, 'timeTotal', toc(t0), ...
    'samples', X(1:n, :));
end
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
