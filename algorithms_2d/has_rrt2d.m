function [path, info] = has_rrt2d(env, opts)
%HAS_RRT2D HAS-RRT（Uwacu et al., IEEE RA-L 2025, 10.1109/LRA.2025.3560878）复现。
%
% 论文全称：Hierarchical Annotated-Skeleton Guided RRT。原算法以工作空间骨架
% （workspace skeleton）的标注作为层次化引导，在骨架指示的通道上优先生长，
% 只在必要时回到局部探索；核心机制（与原文 Algorithm 1/2 一致）：
%   0) daws = DirectAndPruneSkeleton(aws, start)：把骨架定向并剪枝到当前查询；
%   1) tree <- {start}；activeRegions <- InitializeRegion(daws, start)；
%   2) 循环：r <- SelectRegion(activeRegions, daws)，其中
%          p_r = e/(|R|+1) + (1-e) w_r / Σ_{r'} w_{r'}，w_r 为该区域的扩展成功率，
%          整个环境也作为一个区域（概率 e/(|R|+1)，此时等价于普通 RRT）；
%      q_rand <- Sample(r)（在区域球内均匀采样）；
%      AttemptExtension(q_rand, tree) -> q_near, q_new；
%      成功：r.AdvanceRegion(daws)（把区域推进到当前骨架边的末端，并在
%            所有后继边的起点新建区域）、r.IncrementSuccess()；
%      失败：r.RetractRegion(daws, q_near, q_new)（区域中心向 q_near/q_new
%            方向回拉一半）、r.IncrementFailure()；
%      r.UpdateWeight()。
%   3) 终止条件为找到路径或资源耗尽（本实现沿用统一预算，并统计首解时刻）。
%
% 说明：原文未公开源码，本实现依据其公开描述在本文统一代码库中复现；
% 骨架由净空场脊线近似（skeleton_build2d），与所有基线公用同一套碰撞检测、
% 参数预算与随机种子序列。骨架不可用（无法给出起点->终点骨架路径）时，
% 算法退化为普通 RRT，与原文"引导不可用时性能与无引导方法相当"的结论一致。
%
% info 字段与 informed_rrtstar2d 保持一致，便于统一统计。

if nargin < 2, opts = struct; end
def = struct('maxIter', 2000, 'step', 4, 'goalBias', 0.05, 'exploreBias', 0.10, ...
    'regionRadius', [], 'safetyMargin', 0, 'skeletonOpts', struct, 'returnFirst', false, ...
    'star', false, 'useEllipse', false, 'rewireRadius', 14, 'ellipseFix', true);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end

b = env.bounds;
diagLen = hypot(b(2) - b(1), b(4) - b(3));
if isempty(opts.regionRadius), opts.regionRadius = 0.06 * diagLen; end
margin = opts.safetyMargin;

% ---- 1) 骨架构建 / 定向 / 剪枝（预规划阶段，单独计时）----
t0 = tic;
sk = skeleton_build2d(env, opts.skeletonOpts);
tPre = toc(t0);

N = opts.maxIter + 2;
X = zeros(N, 2); P = zeros(N, 1); C = inf(N, 1);
X(1, :) = env.start; C(1) = 0; n = 1;
best = inf; goalIdx = 0;
hist = zeros(8192, 2); nh = 0; firstSolTime = NaN;

% ---- 2) 采样区域初始化 ----
regions = struct('center', {}, 'vi', {}, 'succ', {}, 'fail', {});
if sk.ok && size(sk.verts, 1) >= 2
    regions(1) = struct('center', sk.verts(1, :), 'vi', 1, 'succ', 0, 'fail', 0);
    rPath = true;
else
    rPath = false;                       % 无引导：整体环境区域
end
tOpt0 = tic;
tWall = tic;
for it = 1:opts.maxIter
    if toc(tWall) > opts.timeLimit, break; end

    % ---- 3) 区域选择 ----
    nR = numel(regions);
    e = opts.exploreBias;
    pEnv = e / (nR + 1);
    useEnv = ~rPath || rand < pEnv;
    if ~useEnv
        w = zeros(1, nR);
        for i = 1:nR
            tot = regions(i).succ + regions(i).fail;
            if tot > 0, w(i) = regions(i).succ / tot; end
        end
        if all(w == 0), w = ones(1, nR); end
        p = e / (nR + 1) + (1 - e) * w / sum(w);
        p = p / sum(p);
        idx = find(rand <= cumsum(p), 1, 'first');
        if isempty(idx), idx = nR; end
    else
        idx = 0;
    end

    % ---- 采样 ----
    if opts.useEllipse && isfinite(best)
        % HAS-RRT*（本文构造的最强变体）：有解后改用 Informed-RRT* 的椭圆采样
        xr = sample_ellipse2(env.start, env.goal, best, b, opts.ellipseFix);
    elseif rand < opts.goalBias
        xr = env.goal;
    elseif idx == 0
        xr = [rand_range(b(1), b(2)), rand_range(b(3), b(4))];
    else
        th = 2 * pi * rand; rr = opts.regionRadius * rand^(1 / 2);
        xr = regions(idx).center + rr * [cos(th), sin(th)];
    end

    % ---- 扩展尝试 ----
    [~, near] = min(sum((X(1:n, :) - xr).^2, 2));
    v = xr - X(near, :);
    nv = norm(v);
    qnew = [];
    if nv > 1e-9
        xn = X(near, :) + min(opts.step, nv) * v / nv;
        if inside2(xn, b) && collisionChecking(X(near, :), xn, env.squareAll, env.round, margin)
            qnew = xn;
        end
    end

    if ~isempty(qnew)
        n = n + 1; X(n, :) = qnew; P(n) = near; C(n) = C(near) + norm(qnew - X(near, :));
        if opts.star
            % RRT* 机制：在 q_new 邻域内选最优父节点并重布线
            d2 = sqrt(sum((X(1:n - 1, :) - qnew).^2, 2));
            nb = find(d2 <= opts.rewireRadius);
            for z = 1:numel(nb)
                j = nb(z);
                if C(j) + d2(j) < C(n) - 1e-12 && ...
                        collisionChecking(X(j, :), qnew, env.squareAll, env.round, margin)
                    P(n) = j; C(n) = C(j) + d2(j);
                end
            end
            for z = 1:numel(nb)
                j = nb(z);
                if C(n) + d2(j) < C(j) - 1e-12 && ...
                        collisionChecking(X(n, :), X(j, :), env.squareAll, env.round, margin)
                    P(j) = n; C(j) = C(n) + d2(j);
                end
            end
        end
        if idx > 0
            regions(idx).succ = regions(idx).succ + 1;
            % AdvanceRegion：推进到当前骨架边的末端，并在后继边起点新建区域
            if regions(idx).vi < size(sk.verts, 1)
                regions(idx).vi = regions(idx).vi + 1;
                regions(idx).center = sk.verts(regions(idx).vi, :);
                br = sk.branch{regions(idx).vi};
                for z = 1:size(br, 1)
                    regions(end + 1) = struct('center', br(z, :), 'vi', regions(idx).vi, ...
                        'succ', 0, 'fail', 0); %#ok<AGROW>
                end
            end
        end
        % ---- 目标连接 ----
        dg = norm(qnew - env.goal);
        if dg < opts.step * 1.5 && collisionChecking(qnew, env.goal, env.squareAll, env.round, margin)
            gc = C(n) + dg;
            if gc < best
                best = gc; goalIdx = n;
                if isnan(firstSolTime), firstSolTime = toc(t0); end
                nh = nh + 1; hist(nh, :) = [toc(t0), best];
                if opts.returnFirst, break; end
            end
        end
    else
        if idx > 0
            regions(idx).fail = regions(idx).fail + 1;
            % RetractRegion：区域中心向 q_near 方向回拉一半
            prevPos = X(near, :);
            regions(idx).center = 0.5 * (prevPos + regions(idx).center);
        end
    end
end
tOpt = toc(tOpt0);
tPlanner = tPre + tOpt;

if goalIdx > 0
    q = goalIdx; path = env.goal;
    while q > 1, path(end + 1, :) = X(q, :); q = P(q); end %#ok<AGROW>
    path(end + 1, :) = env.start; path = flipud(path);
else
    path = zeros(0, 2);
end
hist = hist(1:nh, :);
info = struct('success', ~isempty(path), 'length', path_length2(path), 'nodes', n, ...
    'iterations', opts.maxIter, 'goalFound', goalIdx > 0, 'seedUsed', sk.ok, ...
    'seedBias', sk.ok, 'firstSolutionTime', firstSolTime, 'bestHistory', hist, ...
    'tPlanner', tPlanner, 'tShortcut', 0, 'timeTotal', toc(t0), ...
    'samples', X(1:n, :), 'skeletonOK', sk.ok, 'skeletonVerts', size(sk.verts, 1), ...
    'skeletonTime', tPre, 'regions', numel(regions), 'star', opts.star, ...
    'useEllipse', opts.useEllipse, 'lenRaw', path_length2(path), 'lenSmooth', NaN, ...
    'smoothOK', false, 'firstSolIter', NaN);
end

function xr = sample_ellipse2(s, g, c, bounds, fixRot)
% 与 informed_rrtstar2d 同一实现的椭圆采样（长轴沿 start->goal）
if nargin < 5, fixRot = true; end
if c <= 0 || ~isfinite(c)
    xr = [bounds(1) + (bounds(2) - bounds(1)) * rand, bounds(3) + (bounds(4) - bounds(3)) * rand];
    return;
end
a = c / 2; d = norm(g - s);
if c <= d
    xr = [bounds(1) + (bounds(2) - bounds(1)) * rand, bounds(3) + (bounds(4) - bounds(3)) * rand];
    return;
end
bb = sqrt(max(c^2 - d^2, eps)) / 2;
u = sqrt(rand); t = 2 * pi * rand;
p = [a * u * cos(t), bb * u * sin(t)];
th = atan2(g(2) - s(2), g(1) - s(1));
R = [cos(th) -sin(th); sin(th) cos(th)];
if fixRot
    xr = (R * p')' + (s + g) / 2;
else
    xr = p * R + (s + g) / 2;
end
end

function x = rand_range(a, b), x = a + (b - a) * rand; end

function ok = inside2(x, b)
ok = x(1) >= b(1) && x(1) <= b(2) && x(2) >= b(3) && x(2) <= b(4);
end

function L = path_length2(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
