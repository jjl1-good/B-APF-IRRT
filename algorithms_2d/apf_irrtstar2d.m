function [path, info] = apf_irrtstar2d(env, opts)
%APF_IRRTSTAR2D APF-IRRT*（Wu et al., Appl. Sci. 2022, 12, 10905）的等价复现。
%
% 该算法是"预启动/混合"文献基线的代表之一，用于回答审稿意见 R2-1(b)：
% 与已发表的 APF + Informed-RRT* 混合算法在数学等价条件下对比。
%
% 与标准 Informed-RRT* 的差别（按原文摘要与结构复现）：
%   1) 把 APF 的虚拟力场引入"搜索树扩展阶段"：每次采样得到的候选点先被
%      合力场（引力 + 有界斥力 + 平衡力触发的切向逃逸）推移一步，再按
%      标准 steer 规则从最近节点扩展，因此采样分布向目标与自由空间集中，
%      而树的拓扑结构、父节点选择与重布线保持 Informed-RRT* 原样；
%   2) 自适应步长选项：净空大时步长按 min(1+d/ρ0, 2) 放大；
%   3) 一旦存在可行解即切换到椭圆采样（与 Informed-RRT* 一致）。
%
% 说明：原文未公开源码，本实现依据其公开描述在本文统一代码库中复现，
% 与所有基线共用同一套碰撞检测、参数预算与随机种子序列。
%
% info 字段与 informed_rrtstar2d 保持一致，便于统一统计。

if nargin < 2, opts = struct; end
def = struct('maxIter', 2000, 'step', 4, 'goalBias', 0.08, 'rewireRadius', 14, ...
    'rho0', 14, 'kAtt', 1.0, 'kRep', 60, 'kTan', 1.0, 'balanceTol', 0.15, ...
    'repClip', 20, 'stepAdapt', false, 'guideGain', 1.0, ...
    'safetyMargin', 0, 'ellipseFix', true);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end

N = opts.maxIter + 2;
X = zeros(N, 2); P = zeros(N, 1); C = inf(N, 1);
X(1, :) = env.start; C(1) = 0; n = 1;
best = inf; goalIdx = 0;
hist = zeros(8192, 2); nh = 0; firstSolTime = NaN;
t0 = tic; tWall = tic;
margin = opts.safetyMargin;

for it = 1:opts.maxIter
    if toc(tWall) > opts.timeLimit, break; end

    % ---- 采样（与原 Informed-RRT* 相同：目标偏置 / 椭圆 / 全域）----
    if rand < opts.goalBias
        xr = env.goal;
    elseif isfinite(best)
        xr = sample_ellipse2(env.start, env.goal, best, env.bounds, opts.ellipseFix);
    else
        xr = [rand_range(env.bounds(1), env.bounds(2)), rand_range(env.bounds(3), env.bounds(4))];
    end

    % ---- APF 引导：用合力场把候选采样点推移一步 ----
    [xn, gOK] = apf_guide2(xr, env, opts, margin);
    if ~gOK, continue; end

    % ---- 标准 steer：最近节点 -> 引导后的候选点 ----
    [~, near] = min(sum((X(1:n, :) - xn).^2, 2));
    p = X(near, :);
    v = xn - p; nv = norm(v);
    if nv < 1e-9, continue; end
    if opts.stepAdapt
        dnear = obstacle_query2d(p, env);
        st = opts.step * min(1 + dnear / opts.rho0, 2);
    else
        st = opts.step;
    end
    xn = p + min(st, nv) * v / nv;

    if ~inside2(xn, env.bounds), continue; end
    if ~collisionChecking(p, xn, env.squareAll, env.round, margin), continue; end

    % ---- 父节点选择与重布线（与原 Informed-RRT* 相同）----
    dist = sqrt(sum((X(1:n, :) - xn).^2, 2));
    cand = find(dist <= opts.rewireRadius);
    [~, ord] = sort(C(cand) + dist(cand));
    cand = cand(ord);
    par = near; cc = C(near) + norm(xn - p);
    for z = 1:numel(cand)
        j = cand(z);
        if C(j) + dist(j) < cc - 1e-12 && collisionChecking(X(j, :), xn, env.squareAll, env.round, margin)
            par = j; cc = C(j) + dist(j);
        end
    end
    n = n + 1; X(n, :) = xn; P(n) = par; C(n) = cc;
    for z = 1:numel(cand)
        j = cand(z);
        if j ~= par && cc + norm(X(j, :) - xn) < C(j) - 1e-12 && ...
                collisionChecking(xn, X(j, :), env.squareAll, env.round, margin)
            P(j) = n; C(j) = cc + norm(X(j, :) - xn);
        end
    end

    % ---- 目标连接 ----
    dg2 = norm(xn - env.goal);
    if dg2 < opts.step * 1.5 && collisionChecking(xn, env.goal, env.squareAll, env.round, margin)
        gc = cc + dg2;
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
else
    path = zeros(0, 2);
end
if ~isempty(path) && isnan(firstSolTime), firstSolTime = toc(t0); end
hist = hist(1:nh, :);

info = struct('success', ~isempty(path), 'length', path_length2(path), 'nodes', n, ...
    'iterations', opts.maxIter, 'goalFound', goalIdx > 0, 'seedUsed', false, ...
    'seedBias', false, 'firstSolutionTime', firstSolTime, 'bestHistory', hist, ...
    'tPlanner', tPlanner, 'tShortcut', 0, 'timeTotal', toc(t0), 'samples', X(1:n, :));
end

function [xn, ok] = apf_guide2(xr, env, opts, margin)
%APF_GUIDE2 用 APF 合力把候选采样点推移 guidedStep，并做可通行性检验。
%   采样点落在障碍物内部时，由斥力法向把它推出最近表面。
ok = false; xn = xr;
[d, nrm] = obstacle_query2d(xr, env);
if d <= 0
    xn = xr + (opts.step * 1.5 + 1e-6) * nrm;
    if ~inside2(xn, env.bounds), return; end
else
    dg = norm(env.goal - xr);
    F = opts.kAtt * (env.goal - xr) / max(dg, eps);              % 引力
    if d < opts.rho0
        w = min((1 / max(d, 1e-6) - 1 / opts.rho0) / max(d^2, 1e-6), opts.repClip);
        F = F + opts.kRep * w * nrm;                             % 有界斥力
        if d < 0.6 * opts.rho0
            proj = abs(dot(opts.kAtt * (env.goal - xr) / max(dg, eps), nrm));
            if proj <= max(opts.balanceTol * opts.kRep * w, 1e-6)
                F = opts.kTan * [-nrm(2), nrm(1)];               % 纯切向逃逸
            end
        end
    end
    nF = norm(F);
    if ~isfinite(nF) || nF < 1e-9, return; end
    xn = xr + opts.guideGain * opts.step * F / nF;
    if ~inside2(xn, env.bounds)
        xn = min(max(xn, env.bounds([1 3])), env.bounds([2 4]));
    end
end
ok = collisionChecking(xr, xn, env.squareAll, env.round, margin);
end

function xr = sample_ellipse2(s, g, c, bounds, fixRot)
%SAMPLE_ELLIPSE2 与 informed_rrtstar2d 相同的椭圆采样（长轴沿 start->goal）。
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
