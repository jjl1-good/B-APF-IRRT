function [path, info] = ge_irrtstar3d(env, opts)
%GE_IRRTSTAR3D 葛超等（电光与控制, 2025, 32(1): 48-53）二维"改进 Informed-RRT*"的三维推广。
%
% 说明：原文只做二维移动机器人实验，本函数把同一套组件按维度无歧义地推广到三维，
% 用于"三类预规划思想方法"的三维对比。三维化的对应关系：
%   · 势场综合值 F = D_N − w·D_P：D_N 用点到最近图元的距离、D_P 用欧氏目标距离；
%     障碍"质量"取体积（长方体 lx·ly·lz、圆柱 πR²h、球 4/3πR³），按最大值归一化；
%   · 曼哈顿距离（式(12) 的目标项）推广为三维 L1 距离；
%   · 双向贪心直连、动态步长、祖节点近路、去冗余 + 三次 B 样条后处理与二维一致；
%   · 有解后切换到三维长球面（prolate hyperspheroid）采样，与 Informed-RRT* 一致。
%
% 其余口径与二维复现完全相同（见 ge_irrtstar2d.m 的说明）。
if nargin < 2, opts = struct; end
def = struct('maxIter', 6000, 'step', 6, 'goalBias', 0.08, 'rewireRadius', 18, ...
    'clearLambda', 0.8, 'massWeight', 2.0, 'apfSample', 'candidates', 'nCand', 5, ...
    'useEllipse', true, 'grandShortcut', true, 'bidirGreedy', true, ...
    'bspline', true, 'bsplinePts', 10, 'safetyMargin', 0, 'ellipseFix', true);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end

b = env.bounds;
diagLen = sqrt((b(2) - b(1))^2 + (b(4) - b(3))^2 + (b(6) - b(5))^2);
nCube = numel(env.cube.axisX);
nCyl = numel(env.cylinder.X);
nSph = numel(env.sphere.X);
masses = [env.cube.lengthx(:) .* env.cube.lengthy(:) .* env.cube.lengthz(:);
          pi * env.cylinder.radius(:).^2 .* env.cylinder.lengthZ(:);
          (4 / 3) * pi * env.sphere.radius(:).^3];
if isempty(masses), masses = 1; end
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
        xr = sample_ellipse3(env.start, env.goal, best, b);
    elseif strcmpi(opts.apfSample, 'candidates')
        Fbest = -inf; xr = [];
        for kk = 1:max(1, opts.nCand)
            if rand < opts.goalBias
                xc = tgt;
            else
                xc = [b(1) + (b(2) - b(1)) * rand, b(3) + (b(4) - b(3)) * rand, ...
                      b(5) + (b(6) - b(5)) * rand];
            end
            Fv = cand_score(xc, env, tgt, diagLen, Mmax, Mbar, masses, nCube, nCyl, opts.massWeight);
            if Fv > Fbest, Fbest = Fv; xr = xc; end
        end
    elseif rand < opts.goalBias
        xr = tgt;
    else
        xr = [b(1) + (b(2) - b(1)) * rand, b(3) + (b(4) - b(3)) * rand, ...
              b(5) + (b(6) - b(5)) * rand];
    end
    if ~inside3(xr, b), continue; end

    % ---------------- 父节点选择（式(12)）+ 动态步长 ----------------
    dd = sqrt(sum((A.X(1:A.n, :) - xr).^2, 2));
    dm = sum(abs(A.X(1:A.n, :) - tgt), 2);
    J = A.C(1:A.n) + dm + dd;
    [~, ord] = sort(J);
    cs = ord(1:min(6, numel(ord)));
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
        if ~inside3(xtry, b), continue; end
        if collisionChecking3D(A.X(j, :), xtry, env)
            xn = xtry; par = j; cc = A.C(j) + norm(xtry - A.X(j, :));
            break;
        end
    end
    if isempty(xn), continue; end

    % ---------------- 祖节点近路 ----------------
    if opts.grandShortcut && A.P(par) > 0
        gp = A.P(par); dgp = norm(A.X(gp, :) - xn);
        if A.C(gp) + dgp < cc - 1e-9 && collisionChecking3D(A.X(gp, :), xn, env)
            par = gp; cc = A.C(gp) + dgp;
        end
    end

    % ---------------- 插入 + 重布线 ----------------
    A.n = A.n + 1; A.X(A.n, :) = xn; A.P(A.n) = par; A.C(A.n) = cc;
    [dc, ~, idc] = obstacle_query3d(xn, env);
    A.D(A.n) = dc; A.M(A.n) = obstacle_mass3(idc, nCube, nCyl, masses, Mbar);
    d2 = sqrt(sum((A.X(1:A.n - 1, :) - xn).^2, 2));
    nb = find(d2 <= opts.rewireRadius);
    for z = 1:numel(nb)
        j = nb(z);
        if j ~= par && cc + d2(j) < A.C(j) - 1e-12 && collisionChecking3D(xn, A.X(j, :), env)
            A.P(j) = A.n; A.C(j) = cc + d2(j);
        end
    end
    T{ti} = A;

    % ---------------- 双向贪心直连 ----------------
    if opts.bidirGreedy
        for q = unique([Bt.n, 1])
            if q < 1 || q > Bt.n, continue; end
            gap = norm(Bt.X(q, :) - xn);
            if gap < 1e-9, continue; end
            if collisionChecking3D(xn, Bt.X(q, :), env)
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

if bestN(1) > 0 && bestN(2) > 0
    pathRaw = [backtrack(T{1}, bestN(1)); flipud(backtrack(T{2}, bestN(2)))];
else
    pathRaw = zeros(0, 3);
end

lenRaw = path_length3(pathRaw);
path = pathRaw; lenSmooth = NaN; smoothOK = false;
if opts.bspline && size(pathRaw, 1) >= 3
    ps = shortcut_path(pathRaw, env, margin);
    pb = bspline_cubic(ps, opts.bsplinePts);
    if polyline_free(pb, env, margin)
        path = pb; lenSmooth = path_length3(pb); smoothOK = true;
    else
        path = ps; lenSmooth = NaN;
    end
end
if ~isempty(path) && isnan(firstSolTime), firstSolTime = toc(t0); firstSolIter = opts.maxIter; end

info = struct('success', ~isempty(path), 'length', path_length3(path), ...
    'nodes', T{1}.n + T{2}.n, 'iterations', opts.maxIter, 'goalFound', bestN(1) > 0, ...
    'firstSolutionTime', firstSolTime, 'firstSolIter', firstSolIter, ...
    'bestHistory', hist(1:nh, :), 'tPlanner', tPlanner, 'tShortcut', 0, ...
    'timeTotal', toc(t0), 'lenRaw', lenRaw, 'lenSmooth', lenSmooth, ...
    'smoothOK', smoothOK, 'seedUsed', false, 'seedBias', false);
end

% ============================ 局部函数 ============================
function t = tree_init(x0)
N = 20000;
t = struct('X', zeros(N, 3), 'P', zeros(N, 1), 'C', inf(N, 1), 'D', zeros(N, 1), ...
           'M', zeros(N, 1), 'n', 1);
t.X(1, :) = x0; t.C(1) = 0;
end

function F = cand_score(x, env, tgt, diagLen, Mmax, Mbar, masses, nCube, nCyl, w)
[d, ~, id] = obstacle_query3d(x, env);
m = obstacle_mass3(id, nCube, nCyl, masses, Mbar);
lp = norm(x - tgt);
F = (m / Mmax) * d / diagLen - w * (Mbar / Mmax) * lp / diagLen;
end

function m = obstacle_mass3(id, nCube, nCyl, masses, Mbar)
if id >= 3000
    idx = nCube + nCyl + (id - 3000);
elseif id >= 2000
    idx = nCube + (id - 2000);
elseif id >= 1 && id <= nCube
    idx = id;
else
    idx = 0;
end
if idx >= 1 && idx <= numel(masses), m = masses(idx); else, m = Mbar; end
end

function xr = sample_ellipse3(s, g, c, bounds)
% 三维长球面（prolate hyperspheroid）均匀采样：单位球内均匀点做线性映射
if c <= 0 || ~isfinite(c)
    xr = rand_point3(bounds); return;
end
cmin = norm(g - s);
if c <= cmin
    xr = rand_point3(bounds); return;
end
a = c / 2; bb = sqrt(max(c^2 - cmin^2, eps)) / 2;
while true                      % 单位球内均匀采样（拒绝法）
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

function seg = backtrack(A, q)
seg = A.X(q, :);
while A.P(q) > 0, q = A.P(q); seg(end + 1, :) = A.X(q, :); end %#ok<AGROW>
seg = flipud(seg);
end

function ps = shortcut_path(p, env, margin)
ps = p(1, :); i = 1;
while i < size(p, 1)
    j = size(p, 1);
    while j > i + 1 && ~collisionChecking3D(p(i, :), p(j, :), env)
        j = j - 1;
    end
    ps(end + 1, :) = p(j, :); %#ok<AGROW>
    i = j;
end
end

function ok = polyline_free(p, env, margin)
ok = true;
for i = 1:size(p, 1) - 1
    if ~collisionChecking3D(p(i, :), p(i + 1, :), env)
        ok = false; return;
    end
end
end

function B = bspline_cubic(path, nPerSpan)
p1 = path(1, :); pe = path(end, :);
Q = [p1; p1; p1; path(2:end - 1, :); pe; pe; pe];
B = zeros((size(Q, 1) - 3) * nPerSpan + 1, 3);
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

function ok = inside3(x, b)
ok = x(1) >= b(1) && x(1) <= b(2) && x(2) >= b(3) && x(2) <= b(4) && ...
     x(3) >= b(5) && x(3) <= b(6);
end

function L = path_length3(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
