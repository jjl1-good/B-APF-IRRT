function [path, info] = apf_irrtstar3d(env, opts)
%APF_IRRTSTAR3D APF-IRRT*（Wu et al., Appl. Sci. 2022, 12, 10905）的三维复现。
%
% 与 apf_irrtstar2d 完全对应：APF 合力场把候选采样点推移一步后再按标准
% steer 规则扩展，树结构 / 父节点选择 / 重布线保持 Informed-RRT* 原样，
% 一旦存在可行解即切换到椭圆采样。
%
% info 字段与 informed_rrtstar3d 保持一致，便于统一统计。

if nargin < 2, opts = struct; end
def = struct('maxIter', 6000, 'step', 6, 'goalBias', 0.12, 'rewireRadius', 18, ...
    'rho0', 20, 'kAtt', 1.0, 'kRep', 60, 'kTan', 1.0, 'balanceTol', 0.15, ...
    'repClip', 20, 'stepAdapt', false, 'guideGain', 1.0, ...
    'safetyMargin', 0, 'ellipseFix', true);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end

N = opts.maxIter + 2;
X = zeros(N, 3); P = zeros(N, 1); C = inf(N, 1);
X(1, :) = env.start; C(1) = 0; n = 1;
best = inf; goalIdx = 0;
hist = zeros(8192, 2); nh = 0; firstSolTime = NaN;
t0 = tic; tWall = tic;
margin = opts.safetyMargin;

for it = 1:opts.maxIter
    if toc(tWall) > opts.timeLimit, break; end

    % ---- 采样：目标偏置 / 椭圆 / 全域 ----
    if rand < opts.goalBias
        xr = env.goal;
    elseif isfinite(best)
        xr = sample_ellipse3(env.start, env.goal, best, env.bounds, opts.ellipseFix);
    else
        xr = [rand_range(env.bounds(1), env.bounds(2)), rand_range(env.bounds(3), env.bounds(4)), ...
            rand_range(env.bounds(5), env.bounds(6))];
    end

    % ---- APF 引导 ----
    [xn, gOK] = apf_guide3(xr, env, opts, margin);
    if ~gOK, continue; end

    % ---- 标准 steer ----
    [~, near] = min(sum((X(1:n, :) - xn).^2, 2));
    p = X(near, :);
    v = xn - p; nv = norm(v);
    if nv < 1e-9, continue; end
    if opts.stepAdapt
        dnear = obstacle_query3d(p, env);
        st = opts.step * min(1 + dnear / opts.rho0, 2);
    else
        st = opts.step;
    end
    xn = p + min(st, nv) * v / nv;
    if ~inside3(xn, env.bounds), continue; end
    if ~collisionChecking3D(p, xn, env, margin), continue; end

    % ---- 父节点选择与重布线 ----
    dist = sqrt(sum((X(1:n, :) - xn).^2, 2));
    cand = find(dist <= opts.rewireRadius);
    [~, ord] = sort(C(cand) + dist(cand));
    cand = cand(ord);
    par = near; cc = C(near) + norm(xn - p);
    for z = 1:numel(cand)
        j = cand(z);
        if C(j) + dist(j) < cc - 1e-12 && collisionChecking3D(X(j, :), xn, env, margin)
            par = j; cc = C(j) + dist(j);
        end
    end
    n = n + 1; X(n, :) = xn; P(n) = par; C(n) = cc;
    for z = 1:numel(cand)
        j = cand(z);
        if j ~= par && cc + norm(X(j, :) - xn) < C(j) - 1e-12 && ...
                collisionChecking3D(xn, X(j, :), env, margin)
            P(j) = n; C(j) = cc + norm(X(j, :) - xn);
        end
    end

    % ---- 目标连接 ----
    dg2 = norm(xn - env.goal);
    if dg2 < opts.step * 1.5 && collisionChecking3D(xn, env.goal, env, margin)
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
    path = zeros(0, 3);
end
if ~isempty(path) && isnan(firstSolTime), firstSolTime = toc(t0); end
hist = hist(1:nh, :);

info = struct('success', ~isempty(path), 'length', path_length3(path), 'nodes', n, ...
    'iterations', opts.maxIter, 'goalFound', goalIdx > 0, 'seedUsed', false, ...
    'seedBias', false, 'firstSolutionTime', firstSolTime, 'bestHistory', hist, ...
    'tPlanner', tPlanner, 'tShortcut', 0, 'timeTotal', toc(t0), 'samples', X(1:n, :));
end

function [xn, ok] = apf_guide3(xr, env, opts, margin)
ok = false; xn = xr;
[dn, nrm] = obstacle_query3d(xr, env);
inFree = collisionChecking3D(xr, xr, env);
if ~inFree
    xn = xr + (opts.step * 1.5 + 1e-6) * nrm;
    if ~inside3(xn, env.bounds), return; end
else
    dg = norm(env.goal - xr);
    F = opts.kAtt * (env.goal - xr) / max(dg, eps);
    if dn < opts.rho0
        w = min((1 / max(dn, 1e-6) - 1 / opts.rho0) / max(dn^2, 1e-6), opts.repClip);
        F = F + opts.kRep * w * nrm;
        if dn < 0.6 * opts.rho0
            Fatt = opts.kAtt * (env.goal - xr) / max(dg, eps);
            proj = abs(dot(Fatt, nrm));
            if proj <= max(opts.balanceTol * opts.kRep * w, 1e-6)
                tau = Fatt - dot(Fatt, nrm) * nrm;          % 切向分量
                nt = norm(tau);
                if nt > 1e-9, F = opts.kTan * tau / nt; end
            end
        end
    end
    nF = norm(F);
    if ~isfinite(nF) || nF < 1e-9, return; end
    xn = xr + opts.guideGain * opts.step * F / nF;
    if ~inside3(xn, env.bounds)
        xn = min(max(xn, env.bounds([1 3 5])), env.bounds([2 4 6]));
    end
end
ok = collisionChecking3D(xr, xn, env, margin);
end

function xr = sample_ellipse3(s, g, c, b, fixRot)
if nargin < 5, fixRot = true; end
if c <= 0 || ~isfinite(c) || c <= norm(g - s)
    xr = [b(1) + (b(2) - b(1)) * rand, b(3) + (b(4) - b(3)) * rand, b(5) + (b(6) - b(5)) * rand];
    return;
end
d = norm(g - s);
a = c / 2; rad = sqrt(max(c^2 - d^2, eps)) / 2;
u = rand^(1 / 3); th = 2 * pi * rand; ph = acos(2 * rand - 1);
pt = [a * u * sin(ph) * cos(th), rad * u * sin(ph) * sin(th), rad * u * cos(ph)];
dir = (g - s) / d;
tmp = [0 0 1];
if abs(dot(dir, tmp)) > 0.9, tmp = [0 1 0]; end
e1 = cross(dir, tmp); e1 = e1 / norm(e1);
e2 = cross(dir, e1);
if fixRot
    xr = (s + g) / 2 + pt(1) * dir + pt(2) * e1 + pt(3) * e2;
else
    xr = (s + g) / 2 + pt(1) * dir + pt(2) * e2 + pt(3) * e1;
end
end

function x = rand_range(a, b), x = a + (b - a) * rand; end

function ok = inside3(p, b)
ok = p(1) >= b(1) && p(1) <= b(2) && p(2) >= b(3) && p(2) <= b(4) && ...
    p(3) >= b(5) && p(3) <= b(6);
end

function L = path_length3(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
