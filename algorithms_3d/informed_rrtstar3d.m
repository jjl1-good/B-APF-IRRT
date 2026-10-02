function [path, info] = informed_rrtstar3d(env, seedPath, opts)
%INFORMED_RRTSTAR3D 增强型 Informed-RRT*（三维，AB-IRRT* 的优化阶段）。
%
% 增强项与二维版本一致（见 informed_rrtstar2d.m）：
%   1) 预规划路径偏置采样（pbias）；
%   2) 椭球知情采样（找到初始解后）；
%   3) 动态重布线半径（n<Nmax/2 取 rmax，否则 rmax/2）；
%   4) anytime 历史 (t, cbest) 与计时拆分。
%
% info 字段：success, length, nodes, iterations, goalFound, seedUsed,
%   firstSolutionTime, bestHistory, tPlanner, tShortcut, timeTotal。

if nargin < 2, seedPath = []; end
if nargin < 3, opts = struct; end
def = struct('maxIter', 6000, 'step', 6, 'goalBias', 0.12, 'rewireRadius', 18, ...
    'pbias', 0.0, 'dynamicRewire', false, 'rewireMax', 30, 'useSeedEllipse', true);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end

N = opts.maxIter + 2;
X = zeros(N, 3); P = zeros(N, 1); C = inf(N, 1);
X(1, :) = env.start; C(1) = 0; n = 1;
best = inf; goalIdx = 0;
hist = zeros(8192, 2); nh = 0;
firstSolTime = NaN;
t0 = tic;

% 预规划种子：可用于（a）初始化采样椭圆代价 c_best；（b）路径点偏置采样。
seedAvailable = ~isempty(seedPath) && size(seedPath, 1) >= 2;
seedUsed = seedAvailable && opts.useSeedEllipse;
mSeed = 0; if seedAvailable, mSeed = size(seedPath, 1); end
if seedUsed
    best = path_length3(seedPath);
    firstSolTime = 0;
    nh = nh + 1; hist(nh, :) = [0, best];
end
seedPtr = 1;

if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end
tWall = tic;                      % 等时间对照用：墙钟上限（默认 Inf 表示不启用）
for it = 1:opts.maxIter
    if toc(tWall) > opts.timeLimit, break; end
    if opts.dynamicRewire
        if it < opts.maxIter / 2
            rw = opts.rewireMax;
        else
            rw = opts.rewireMax / 2;
        end
    else
        rw = opts.rewireRadius;
    end

    useBias = opts.pbias > 0 && seedPtr <= mSeed && rand < opts.pbias;
    if useBias
        xs = seedPath(seedPtr, :);
        seedPtr = seedPtr + 1;
        [~, near] = min(sum((X(1:n, :) - xs).^2, 2));
        v = xs - X(near, :);
        if norm(v) < eps, continue; end
        xn = X(near, :) + min(opts.step, norm(v)) * v / norm(v);
    else
        if rand < opts.goalBias
            xr = env.goal;
        elseif isfinite(best)
            xr = sample3(env.start, env.goal, best, env.bounds);
        else
            xr = [randb(env.bounds(1), env.bounds(2)), randb(env.bounds(3), env.bounds(4)), ...
                  randb(env.bounds(5), env.bounds(6))];
        end
        [~, near] = min(sum((X(1:n, :) - xr).^2, 2));
        v = xr - X(near, :);
        if norm(v) < eps, continue; end
        xn = X(near, :) + min(opts.step, norm(v)) * v / norm(v);
    end

    if ~inside3(xn, env.bounds) || ~collisionChecking3D(X(near, :), xn, env), continue; end

    dist = sqrt(sum((X(1:n, :) - xn).^2, 2));
    cand = find(dist <= rw);
    par = near; cc = C(near) + norm(xn - X(near, :));
    [~, ord] = sort(C(cand) + dist(cand));
    cand = cand(ord);
    for z = 1:numel(cand)
        j = cand(z);
        if C(j) + dist(j) < cc - 1e-12 && collisionChecking3D(X(j, :), xn, env)
            par = j; cc = C(j) + dist(j);
        end
    end
    n = n + 1; X(n, :) = xn; P(n) = par; C(n) = cc;
    for z = 1:numel(cand)
        j = cand(z);
        if j ~= par && cc + norm(X(j, :) - xn) < C(j) - 1e-12 && collisionChecking3D(xn, X(j, :), env)
            P(j) = n; C(j) = cc + norm(X(j, :) - xn);
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
elseif seedUsed
    path = seedPath; tShortcut = 0;
else
    path = zeros(0, 3); tShortcut = 0;
end
if ~isempty(path) && size(path, 1) >= 2 && isnan(firstSolTime), firstSolTime = toc(t0); end

hist = hist(1:nh, :);
info = struct('success', ~isempty(path), 'length', path_length3(path), 'nodes', n, ...
    'iterations', opts.maxIter, 'goalFound', goalIdx > 0, 'seedUsed', seedUsed, ...
    'seedBias', mSeed > 0 && opts.pbias > 0, ...
    'firstSolutionTime', firstSolTime, 'bestHistory', hist, ...
    'tPlanner', tPlanner, 'tShortcut', tShortcut, 'timeTotal', toc(t0), ...
    'samples', X(1:n, :));
end

function xr = sample3(s, g, c, b)
d = norm(g - s);
if c <= d
    xr = [randb(b(1), b(2)), randb(b(3), b(4)), randb(b(5), b(6))];
    return;
end
a = c / 2; bb = sqrt(max(c^2 - d^2, eps)) / 2;
r = rand^(1/3);
u = randn(1, 3); u = u / norm(u);
z = [a * r * u(1), bb * r * u(2), bb * r * u(3)];
e = (g - s) / d;
ref = [0 0 1];
if abs(dot(e, ref)) > 0.9, ref = [0 1 0]; end
v = cross(e, ref); v = v / norm(v);
w = cross(e, v);
xr = (s + g) / 2 + z(1) * e + z(2) * v + z(3) * w;
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
