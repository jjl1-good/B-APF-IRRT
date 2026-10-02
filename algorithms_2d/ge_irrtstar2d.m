function [path, info] = ge_irrtstar2d(env, opts)
%GE_IRRTSTAR2D 葛超等（电光与控制, 2025, 32(1): 48-53）"改进 Informed-RRT*"的等价复现（二维）。
%
% 复现原文的组件（正文 §2.1–§2.5 与 §3）：
%   1) 基于人工势场法的选点策略（§2.1）：对树上节点 i
%        D_N(i) = M_i/M_max · l_n(i)/√(X²+Y²)     l_n：到最近障碍的距离
%        D_P(i) = M̄/M_max · l_p(i)/√(X²+Y²)       l_p：到该树目标的距离
%        F(i)   = D_N(i) − w·D_P(i)               原文 w = 2（式(6)）
%        F̄(i)   = (F(i)−F_min)/(F_max−F_min)，P(i) = F̄(i)/ΣF̄（式(7)）
%      按 P(i) 抽"优质节点"，并在其邻域内取本步采样点。原文只给出概率公式、
%      未给出该步的精确取点方式，故此处按"优质节点邻域取样"实现；
%      opts.apfSample='uniform' 时退回全域均匀采样（仅保留式(12)的父节点选择），
%      用于口径敏感性对照。
%   2) 动态步长（§2.2）：s = s_base + λ·d̄(i)（d̄ 为净空在树上的归一化值，式(8)–(11)）。
%   3) 双向贪心直连（§2.3）：起点树/目标树交替生长，新节点与另一棵树的最新节点及根做直连检测。
%   4) 启发式扩展（§2.4）：父节点 q_near = argmin[ C(q,root) + d(q,target) + d(q,q_rand) ]（式(12)）。
%   5) 祖节点近路（§2.5）：q_new 与 q_near 的祖节点直连且代价更低时改接祖节点。
%   6) 后处理（§3）：去除冗余节点 + 三次均匀 B 样条平滑；平滑后逐段复检，碰撞则回退。
%
% info 额外字段：lenRaw（树输出折线长度）、lenSmooth（B 样条长度）、smoothOK、firstSolIter。

if nargin < 2, opts = struct; end
def = struct('maxIter', 2000, 'step', 4, 'goalBias', 0.08, 'rewireRadius', 14, ...
    'clearLambda', 0.8, 'massWeight', 2.0, 'apfSample', 'candidates', 'nCand', 5, ...
    'useEllipse', true, 'grandShortcut', true, 'bidirGreedy', true, ...
    'bspline', true, 'bsplinePts', 12, 'safetyMargin', 0, 'ellipseFix', true);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end

b = env.bounds;
diagLen = sqrt((b(2) - b(1))^2 + (b(4) - b(3))^2);
nRect = size(env.square, 1);
masses = [env.square(:, 3) .* env.square(:, 4); pi * env.round(:, 3).^2];
Mmax = max(masses); Mbar = mean(masses);
margin = opts.safetyMargin;

T = {tree_init(env.start), tree_init(env.goal)};
targetOf = {env.goal, env.start};

best = inf; bestN = [0 0]; hist = zeros(8192, 2); nh = 0;
firstSolTime = NaN; firstSolIter = NaN;
t0 = tic; tWall = tic;

for it = 1:opts.maxIter
    if toc(tWall) > opts.timeLimit, break; end
    ti = mod(it - 1, 2) + 1; tj = 3 - ti;
    A = T{ti}; Bt = T{tj};
    tgt = targetOf{ti};

    % ---------------- 采样 ----------------
    if opts.useEllipse && isfinite(best)
        xr = sample_ellipse2(env.start, env.goal, best, b, opts.ellipseFix);
    elseif strcmpi(opts.apfSample, 'candidates')
        % §2.1：生成若干候选采样点，用势场综合值 F = D_N − w·D_P 筛选"优质采样点"
        % （原文图 3 即按此给出若干候选点的 F 值）。
        Fbest = -inf; xr = [];
        for kk = 1:max(1, opts.nCand)
            if rand < opts.goalBias
                xc = tgt;
            else
                xc = [b(1) + (b(2) - b(1)) * rand, b(3) + (b(4) - b(3)) * rand];
            end
            Fv = cand_score(xc, env, tgt, diagLen, Mmax, Mbar, masses, nRect, opts.massWeight);
            if Fv > Fbest, Fbest = Fv; xr = xc; end
        end
    elseif strcmpi(opts.apfSample, 'apf') && A.n >= 2
        P = apf_prob(A, tgt, diagLen, Mmax, Mbar, opts.massWeight);
        idx = draw_from(P);
        rad = opts.step * (0.5 + 1.5 * rand);
        th = 2 * pi * rand;
        xr = A.X(idx, :) + rad * [cos(th) sin(th)];
        xr = min(max(xr, b([1 3])), b([2 4]));
    elseif rand < opts.goalBias
        xr = tgt;
    else
        xr = [b(1) + (b(2) - b(1)) * rand, b(3) + (b(4) - b(3)) * rand];
    end
    if ~inside2(xr, b), continue; end

    % ---------------- 父节点选择（式(12)）+ 动态步长（式(11)） ----------------
    dd = sqrt(sum((A.X(1:A.n, :) - xr).^2, 2));
    dm = abs(A.X(1:A.n, 1) - tgt(1)) + abs(A.X(1:A.n, 2) - tgt(2));
    J = A.C(1:A.n) + dm + dd;
    [~, ord] = sort(J);
    if strcmpi(opts.apfSample, 'node') && A.n >= 2
        % §2.1 的"节点被选为采样点（扩展基点）"读法：按势场概率 P(i) 抽扩展基点
        P = apf_prob(A, tgt, diagLen, Mmax, Mbar, opts.massWeight);
        cs = zeros(3, 1);
        for z = 1:3, cs(z) = draw_from(P); end
    else
        cs = ord(1:min(6, numel(ord)));
    end
    if A.n > 1
        dmin = min(A.D(1:A.n)); dmax = max(A.D(1:A.n));
    else
        dmin = 0; dmax = 1;
    end

    xn = []; par = 0; cc = inf;
    for z = 1:numel(cs)
        j = cs(z);
        v = xr - A.X(j, :); nv = norm(v);
        if nv < 1e-9, continue; end
        dbar = 0; if dmax - dmin > 1e-12, dbar = (A.D(j) - dmin) / (dmax - dmin); end
        s = opts.step + opts.clearLambda * dbar;
        xtry = A.X(j, :) + min(s, nv) * v / nv;
        if ~inside2(xtry, b), continue; end
        if collisionChecking(A.X(j, :), xtry, env.squareAll, env.round, margin)
            xn = xtry; par = j; cc = A.C(j) + norm(xtry - A.X(j, :));
            break;
        end
    end
    if isempty(xn), continue; end

    % ---------------- 祖节点近路（§2.5） ----------------
    if opts.grandShortcut && A.P(par) > 0
        gp = A.P(par); dgp = norm(A.X(gp, :) - xn);
        if A.C(gp) + dgp < cc - 1e-9 && ...
                collisionChecking(A.X(gp, :), xn, env.squareAll, env.round, margin)
            par = gp; cc = A.C(gp) + dgp;
        end
    end

    % ---------------- 插入 + 重布线 ----------------
    A.n = A.n + 1; A.X(A.n, :) = xn; A.P(A.n) = par; A.C(A.n) = cc;
    [dc, ~, idc] = obstacle_query2d(xn, env);
    A.D(A.n) = dc; A.M(A.n) = obstacle_mass(idc, nRect, masses, Mbar);
    d2 = sqrt(sum((A.X(1:A.n - 1, :) - xn).^2, 2));
    nb = find(d2 <= opts.rewireRadius);
    for z = 1:numel(nb)
        j = nb(z);
        if j ~= par && cc + d2(j) < A.C(j) - 1e-12 && ...
                collisionChecking(xn, A.X(j, :), env.squareAll, env.round, margin)
            A.P(j) = A.n; A.C(j) = cc + d2(j);
        end
    end
    T{ti} = A;

    % ---------------- 双向贪心直连（§2.3） ----------------
    if opts.bidirGreedy
        for q = unique([Bt.n, 1])
            if q < 1 || q > Bt.n, continue; end
            gap = norm(Bt.X(q, :) - xn);
            if gap < 1e-9, continue; end
            if collisionChecking(xn, Bt.X(q, :), env.squareAll, env.round, margin)
                cnew = cc + Bt.C(q) + gap;
                if cnew < best
                    best = cnew;
                    if ti == 1, bestN = [A.n, q]; else, bestN = [q, A.n]; end
                    if isnan(firstSolTime), firstSolTime = toc(t0); firstSolIter = it; end
                    nh = nh + 1; hist(nh, :) = [toc(t0), best];
                end
            end
        end
    end
end
tPlanner = toc(t0);

% ---------------- 原始折线 ----------------
if bestN(1) > 0 && bestN(2) > 0
    pathRaw = [backtrack(T{1}, bestN(1)); flipud(backtrack(T{2}, bestN(2)))];
else
    pathRaw = zeros(0, 2);
end

% ---------------- 后处理：去冗余 + 三次 B 样条（§3） ----------------
lenRaw = path_length2(pathRaw);
path = pathRaw; lenSmooth = NaN; smoothOK = false;
if opts.bspline && size(pathRaw, 1) >= 3
    ps = shortcut_path(pathRaw, env, margin);
    pb = bspline_cubic(ps, opts.bsplinePts);
    if polyline_free(pb, env, margin)
        path = pb; lenSmooth = path_length2(pb); smoothOK = true;
    else
        path = ps; lenSmooth = NaN;      % 平滑路径越障：回退为去冗余折线（不改变长度）
    end
end
if ~isempty(path) && isnan(firstSolTime), firstSolTime = toc(t0); firstSolIter = opts.maxIter; end

info = struct('success', ~isempty(path), 'length', path_length2(path), ...
    'nodes', T{1}.n + T{2}.n, 'iterations', opts.maxIter, 'goalFound', bestN(1) > 0, ...
    'firstSolutionTime', firstSolTime, 'firstSolIter', firstSolIter, ...
    'bestHistory', hist(1:nh, :), 'tPlanner', tPlanner, 'tShortcut', 0, ...
    'timeTotal', toc(t0), 'lenRaw', lenRaw, 'lenSmooth', lenSmooth, ...
    'smoothOK', smoothOK, 'seedUsed', false, 'seedBias', false);
end

% ============================ 局部函数 ============================
function t = tree_init(x0)
N = 20000;
t = struct('X', zeros(N, 2), 'P', zeros(N, 1), 'C', inf(N, 1), 'D', zeros(N, 1), ...
           'M', zeros(N, 1), 'n', 1);
t.X(1, :) = x0; t.C(1) = 0;
end

function F = cand_score(x, env, tgt, diagLen, Mmax, Mbar, masses, nRect, w)
% 候选采样点的势场综合值（式(4)–(6)）：F = D_N − w·D_P
[d, ~, id] = obstacle_query2d(x, env);
m = obstacle_mass(id, nRect, masses, Mbar);
lp = norm(x - tgt);
F = (m / Mmax) * d / diagLen - w * (Mbar / Mmax) * lp / diagLen;
end

function P = apf_prob(A, tgt, diagLen, Mmax, Mbar, wDp)
n = A.n;
lp = sqrt(sum((A.X(1:n, :) - tgt).^2, 2));
DN = (A.M(1:n) / Mmax) .* A.D(1:n) / diagLen;
DP = (Mbar / Mmax) * lp / diagLen;
F = DN - wDp * DP;
lo = min(F); hi = max(F);
if hi - lo < 1e-12
    P = ones(n, 1) / n;
else
    Fb = (F - lo) / (hi - lo);
    s = sum(Fb);
    if s <= 0, P = ones(n, 1) / n; else, P = Fb / s; end
end
end

function m = obstacle_mass(id, nRect, masses, Mbar)
% id：1..nRect 为真实矩形，其后为边界墙；1000+k 为第 k 个圆
if id >= 1000
    k = id - 1000;
    if k >= 1 && k <= numel(masses) - nRect
        m = masses(nRect + k);
    else
        m = Mbar;
    end
elseif id >= 1 && id <= nRect
    m = masses(id);
else
    m = Mbar;                       % 边界墙按中性质量处理
end
end

function idx = draw_from(P)
c = cumsum(P); c(end) = 1;
idx = find(rand <= c, 1, 'first');
if isempty(idx), idx = numel(P); end
end

function xr = sample_ellipse2(s, g, c, bounds, fixRot)
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

function seg = backtrack(A, q)
seg = A.X(q, :);
while A.P(q) > 0, q = A.P(q); seg(end + 1, :) = A.X(q, :); end %#ok<AGROW>
seg = flipud(seg);
end

function ps = shortcut_path(p, env, margin)
ps = p(1, :); i = 1;
while i < size(p, 1)
    j = size(p, 1);
    while j > i + 1 && ~collisionChecking(p(i, :), p(j, :), env.squareAll, env.round, margin)
        j = j - 1;
    end
    ps(end + 1, :) = p(j, :); %#ok<AGROW>
    i = j;
end
end

function ok = polyline_free(p, env, margin)
ok = true;
for i = 1:size(p, 1) - 1
    if ~collisionChecking(p(i, :), p(i + 1, :), env.squareAll, env.round, margin)
        ok = false; return;
    end
end
end

function B = bspline_cubic(path, nPerSpan)
p1 = path(1, :); pe = path(end, :);
Q = [p1; p1; p1; path(2:end - 1, :); pe; pe; pe];
B = zeros((size(Q, 1) - 3) * nPerSpan + 1, 2);
z = 0;
for i = 1:size(Q, 1) - 3
    q0 = Q(i, :); q1 = Q(i + 1, :); q2 = Q(i + 2, :); q3 = Q(i + 3, :);
    for k = 0:nPerSpan - 1
        t = k / nPerSpan;
        b0 = (1 - t)^3 / 6; b1 = (3 * t^3 - 6 * t^2 + 4) / 6;
        b2 = (-3 * t^3 + 3 * t^2 + 3 * t + 1) / 6; b3 = t^3 / 6;
        z = z + 1; B(z, :) = b0 * q0 + b1 * q1 + b2 * q2 + b3 * q3;
    end
end
z = z + 1; B(z, :) = pe;
B = B(1:z, :);
end

function ok = inside2(x, b)
ok = x(1) >= b(1) && x(1) <= b(2) && x(2) >= b(3) && x(2) <= b(4);
end

function L = path_length2(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
