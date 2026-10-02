function [path, info] = informed_rrtstar2d(env, seedPath, opts)
%INFORMED_RRTSTAR2D 增强型 Informed-RRT*（二维，AB-IRRT* 的优化阶段）。
%
% 相对标准 Informed-RRT* 的增强（与论文 2.5 节一一对应）：
%   1) 预规划路径偏置采样（pbias）：以概率 pbias 直接尝试把预规划路径
%      点接入树（best parent 选择 + 精确碰撞检测），加速可行解融入；
%   2) 自由空间采样：非偏置时在 C_free 内采样（椭球或全域）；
%   3) 动态重布线半径：n<Nmax/2 时取 rmax，之后取 rmax/2，
%      兼顾早期收敛速度与后期计算效率；
%   4) 全程记录 (时间, 最优代价) 历史，用于 anytime 曲线统计。
%
% 输入 seedPath 为 Bug-APF 预规划路径（可空）。当 seedPath 非空时，
% 初始最优代价 cbest 直接取自预规划路径长度，采样椭圆立即启用。
%
% info 字段：success, length, nodes, iterations, goalFound,
%   firstSolutionTime, bestHistory, tPlanner, tShortcut, timeTotal,
%   seedUsed（是否使用预规划种子）。

if nargin < 2, seedPath = []; end
if nargin < 3, opts = struct; end
def = struct('maxIter', 6000, 'step', 4, 'goalBias', 0.12, 'rewireRadius', 14, ...
    'pbias', 0.0, 'dynamicRewire', false, 'rewireMax', 16, 'useSeedEllipse', true, ...
    'ellipseFix', true);
% ellipseFix: true = 采样椭圆长轴严格沿 start->goal（正确实现）；
%             false = 保留早期实现的旋转写法（长轴被镜像/转向），仅用于复现旧结果。
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end

N = opts.maxIter + 2;
X = zeros(N, 2); P = zeros(N, 1); C = inf(N, 1);
X(1, :) = env.start; C(1) = 0; n = 1;
best = inf; goalIdx = 0;
hist = zeros(8192, 2); nh = 0;
firstSolTime = NaN;
t0 = tic;

% 预规划种子：可用于（a）初始化采样椭圆代价 c_best；（b）路径点偏置采样。
% useSeedEllipse=false 时保留偏置采样但不用种子初始化椭圆（消融研究用）。
seedAvailable = ~isempty(seedPath) && size(seedPath, 1) >= 2;
seedUsed = seedAvailable && opts.useSeedEllipse;
mSeed = 0; if seedAvailable, mSeed = size(seedPath, 1); end
if seedUsed
    best = path_length2d(seedPath);
    firstSolTime = 0;
    nh = nh + 1; hist(nh, :) = [0, best];
end
seedPtr = 1;                       % 预规划路径点指针

if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end
tWall = tic;                      % 等时间对照用：墙钟上限（默认 Inf 表示不启用）
for it = 1:opts.maxIter
    if toc(tWall) > opts.timeLimit, break; end
    % 动态重布线半径
    if opts.dynamicRewire
        if it < opts.maxIter / 2
            rw = opts.rewireMax;
        else
            rw = opts.rewireMax / 2;
        end
    else
        rw = opts.rewireRadius;
    end

    % ---- 采样 ----
    useBias = opts.pbias > 0 && seedPtr <= mSeed && rand < opts.pbias;
    if useBias
        xs = seedPath(seedPtr, :);
        seedPtr = seedPtr + 1;
        [~, near] = min(sum((X(1:n, :) - xs).^2, 2));
        d = xs - X(near, :);
        if norm(d) < eps, continue; end
        xn = X(near, :) + min(opts.step, norm(d)) * d / norm(d);
    else
        if rand < opts.goalBias
            xr = env.goal;
        elseif isfinite(best)
            xr = sample_ellipse(env.start, env.goal, best, env.bounds, opts.ellipseFix);
        else
            xr = [rand_range(env.bounds(1), env.bounds(2)), rand_range(env.bounds(3), env.bounds(4))];
        end
        [~, near] = min(sum((X(1:n, :) - xr).^2, 2));
        d = xr - X(near, :);
        if norm(d) < eps, continue; end
        xn = X(near, :) + min(opts.step, norm(d)) * d / norm(d);
    end

    if ~inside(xn, env.bounds) || ~collisionChecking(X(near, :), xn, env.squareAll, env.round), continue; end

    % ---- 最优父节点选择与重布线 ----
    dist = sqrt(sum((X(1:n, :) - xn).^2, 2));
    cand = find(dist <= rw);
    [~, ord] = sort(C(cand) + dist(cand));
    cand = cand(ord);
    par = near; cc = C(near) + norm(xn - X(near, :));
    for z = 1:numel(cand)
        j = cand(z);
        if C(j) + dist(j) < cc - 1e-12 && collisionChecking(X(j, :), xn, env.squareAll, env.round)
            par = j; cc = C(j) + dist(j);
        end
    end
    n = n + 1; X(n, :) = xn; P(n) = par; C(n) = cc;
    for z = 1:numel(cand)
        j = cand(z);
        if j ~= par && cc + norm(X(j, :) - xn) < C(j) - 1e-12 && collisionChecking(xn, X(j, :), env.squareAll, env.round)
            P(j) = n; C(j) = cc + norm(X(j, :) - xn);
        end
    end

    % ---- 目标连接与上界更新 ----
    dg = norm(xn - env.goal);
    if dg < opts.step * 1.5 && collisionChecking(xn, env.goal, env.squareAll, env.round)
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
    path = zeros(0, 2); tShortcut = 0;
end
if ~isempty(path) && size(path, 1) >= 2 && isnan(firstSolTime), firstSolTime = toc(t0); end

hist = hist(1:nh, :);
info = struct('success', ~isempty(path), 'length', path_length2d(path), 'nodes', n, ...
    'iterations', opts.maxIter, 'goalFound', goalIdx > 0, 'seedUsed', seedUsed, ...
    'seedBias', mSeed > 0 && opts.pbias > 0, ...
    'firstSolutionTime', firstSolTime, 'bestHistory', hist, ...
    'tPlanner', tPlanner, 'tShortcut', tShortcut, 'timeTotal', toc(t0), ...
    'samples', X(1:n, :));
end

function xr = sample_ellipse(s, g, c, bounds, fixRot)
%SAMPLE_ELLIPSE 在 informed 椭球内均匀采样。
% 椭球定义为 {x : |x-s| + |x-g| <= c}，中心为 (s+g)/2，
% 长半轴 a = c/2 沿 start->goal 方向，短半轴 b = sqrt(c^2-d^2)/2。
% fixRot=false 时保留旧实现（p*R），其长轴被镜像到 start->goal 的
% 镜像方向；当 start->goal 为对角线时两者正交，采样区域与真 informed
% 集几乎不重叠（详见 check_ellipse_orientation.m）。
if nargin < 5, fixRot = true; end
if c <= 0 || ~isfinite(c)
    xr = [rand_range(bounds(1), bounds(2)), rand_range(bounds(3), bounds(4))];
    return;
end
a = c / 2; d = norm(g - s);
if c <= d
    xr = [rand_range(bounds(1), bounds(2)), rand_range(bounds(3), bounds(4))];
    return;
end
b = sqrt(max(c^2 - d^2, eps)) / 2;
u = sqrt(rand); t = 2 * pi * rand;
p = [a * u * cos(t), b * u * sin(t)];
th = atan2(g(2) - s(2), g(1) - s(1));
R = [cos(th) -sin(th); sin(th) cos(th)];       % 逆时针旋转到 start->goal
if fixRot
    xr = (R * p')' + (s + g) / 2;             % 正确：长轴沿 start->goal
else
    xr = p * R + (s + g) / 2;                 % 旧实现：行向量右乘 => 旋转取反复共轭
end
end

function x = rand_range(a, b), x = a + (b - a) * rand; end

function ok = inside(x, b)
ok = x(1) >= b(1) && x(1) <= b(2) && x(2) >= b(3) && x(2) <= b(4);
end

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
