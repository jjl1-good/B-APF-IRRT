function [path, info] = has_rrt3d(env, opts)
%HAS_RRT3D HAS-RRT（Uwacu et al., IEEE RA-L 2025, 10.1109/LRA.2025.3560878）三维复现。
%
% 与 has_rrt2d 完全对应：骨架构建/定向/剪枝 -> 区域初始化 -> 区域选择
% （p_r = e/(|R|+1) + (1-e) w_r / Σ w_r'，整体环境另行以 e/(|R|+1) 概率被选中）
% -> 区域内采样 -> RRT 扩展 -> 成功推进区域 / 失败回拉半个距离并记失败。
%
% info 字段与 informed_rrtstar3d 保持一致，便于统一统计。

if nargin < 2, opts = struct; end
def = struct('maxIter', 6000, 'step', 6, 'goalBias', 0.05, 'exploreBias', 0.10, ...
    'regionRadius', [], 'safetyMargin', 0, 'skeletonOpts', struct, 'returnFirst', false, ...
    'star', false, 'useEllipse', false, 'rewireRadius', 18);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end

b = env.bounds;
diagLen = norm([b(2) - b(1), b(4) - b(3), b(6) - b(5)]);
if isempty(opts.regionRadius), opts.regionRadius = 0.06 * diagLen; end
margin = opts.safetyMargin;

% ---- 1) 骨架构建 / 定向 / 剪枝 ----
t0 = tic;
sk = skeleton_build3d(env, opts.skeletonOpts);
tPre = toc(t0);

N = opts.maxIter + 2;
X = zeros(N, 3); P = zeros(N, 1); C = inf(N, 1);
X(1, :) = env.start; C(1) = 0; n = 1;
best = inf; goalIdx = 0;
hist = zeros(8192, 2); nh = 0; firstSolTime = NaN;

regions = struct('center', {}, 'vi', {}, 'succ', {}, 'fail', {});
if sk.ok && size(sk.verts, 1) >= 2
    regions(1) = struct('center', sk.verts(1, :), 'vi', 1, 'succ', 0, 'fail', 0);
    rPath = true;
else
    rPath = false;
end

tOpt0 = tic; tWall = tic;
for it = 1:opts.maxIter
    if toc(tWall) > opts.timeLimit, break; end

    % ---- 区域选择 ----
    nR = numel(regions);
    e = opts.exploreBias;
    useEnv = ~rPath || rand < e / (nR + 1);
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
        % HAS-RRT*（本文构造的最强变体）：有解后改用 Informed 长球面采样
        xr = sample_ellipse3(env.start, env.goal, best, b);
    elseif rand < opts.goalBias
        xr = env.goal;
    elseif idx == 0
        xr = [b(1) + (b(2) - b(1)) * rand, b(3) + (b(4) - b(3)) * rand, b(5) + (b(6) - b(5)) * rand];
    else
        v = randn(1, 3); v = v / max(norm(v), 1e-12);
        xr = regions(idx).center + opts.regionRadius * rand^(1 / 3) * v;
    end

    % ---- 扩展尝试 ----
    [~, near] = min(sum((X(1:n, :) - xr).^2, 2));
    v = xr - X(near, :); nv = norm(v);
    qnew = [];
    if nv > 1e-9
        xn = X(near, :) + min(opts.step, nv) * v / nv;
        if inside3(xn, b) && collisionChecking3D(X(near, :), xn, env, margin)
            qnew = xn;
        end
    end

    if ~isempty(qnew)
        n = n + 1; X(n, :) = qnew; P(n) = near; C(n) = C(near) + norm(qnew - X(near, :));
        if opts.star
            % RRT* 机制：q_new 邻域内选最优父节点并重布线
            d2 = sqrt(sum((X(1:n - 1, :) - qnew).^2, 2));
            nb = find(d2 <= opts.rewireRadius);
            for z = 1:numel(nb)
                j = nb(z);
                if C(j) + d2(j) < C(n) - 1e-12 && collisionChecking3D(X(j, :), qnew, env, margin)
                    P(n) = j; C(n) = C(j) + d2(j);
                end
            end
            for z = 1:numel(nb)
                j = nb(z);
                if C(n) + d2(j) < C(j) - 1e-12 && collisionChecking3D(X(n, :), X(j, :), env, margin)
                    P(j) = n; C(j) = C(n) + d2(j);
                end
            end
        end
        if idx > 0
            regions(idx).succ = regions(idx).succ + 1;
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
        dg = norm(qnew - env.goal);
        if dg < opts.step * 1.5 && collisionChecking3D(qnew, env.goal, env, margin)
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
            regions(idx).center = 0.5 * (X(near, :) + regions(idx).center);
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
    path = zeros(0, 3);
end
hist = hist(1:nh, :);
info = struct('success', ~isempty(path), 'length', path_length3(path), 'nodes', n, ...
    'iterations', opts.maxIter, 'goalFound', goalIdx > 0, 'seedUsed', sk.ok, ...
    'seedBias', sk.ok, 'firstSolutionTime', firstSolTime, 'bestHistory', hist, ...
    'tPlanner', tPlanner, 'tShortcut', 0, 'timeTotal', toc(t0), ...
    'samples', X(1:n, :), 'skeletonOK', sk.ok, 'skeletonVerts', size(sk.verts, 1), ...
    'skeletonTime', tPre, 'regions', numel(regions), 'star', opts.star, ...
    'useEllipse', opts.useEllipse, 'lenRaw', path_length3(path), 'lenSmooth', NaN, ...
    'smoothOK', false, 'firstSolIter', NaN);
end

function xr = sample_ellipse3(s, g, c, bounds)
% 三维长球面均匀采样（与 ge_irrtstar3d 同一实现）
if c <= 0 || ~isfinite(c)
    xr = rand_point3(bounds); return;
end
cmin = norm(g - s);
if c <= cmin
    xr = rand_point3(bounds); return;
end
a = c / 2; bb = sqrt(max(c^2 - cmin^2, eps)) / 2;
while true
    p = 2 * rand(1, 3) - 1;
    if norm(p) <= 1, break; end
end
dir = (g - s) / max(cmin, eps);
[e1, e2] = orthonormal_basis(dir);
xr = (s + g) / 2 + a * p(1) * dir + bb * p(2) * e1 + bb * p(3) * e2;
end

function x = rand_point3(bounds)
x = [bounds(1) + (bounds(2) - bounds(1)) * rand, bounds(3) + (bounds(4) - bounds(3)) * rand, ...
     bounds(5) + (bounds(6) - bounds(5)) * rand];
end

function [e1, e2] = orthonormal_basis(d)
tmp = [1 0 0]; if abs(dot(d, tmp)) > 0.9, tmp = [0 1 0]; end
e1 = cross(d, tmp); e1 = e1 / norm(e1);
e2 = cross(d, e1);
end

function ok = inside3(p, b)
ok = p(1) >= b(1) && p(1) <= b(2) && p(2) >= b(3) && p(2) <= b(4) && ...
    p(3) >= b(5) && p(3) <= b(6);
end

function L = path_length3(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
