function [path, info] = stable_apf3d(env, opts)
%STABLE_APF3D 三维 Bug-APF 预规划器（形式化状态机 + 三维方向判定规则）。
%
% 与论文 2.4 节的对应关系（可执行定义）：
%   S1 DIRECT : I_direct=1（对目标视线无遮挡）→ 纯吸引场 + 自适应步长
%   S2 AVOID  : 视线受阻且 d<ρ0 → F=F_att+β(F_rep+F_tex)
%   S3 ESCAPE : 平衡力检测 |proj_n(F_att)|≈||F_rep|| 且 d<0.6ρ0
%               → 纯切向逃逸方向（仍逐步执行精确碰撞检测）
%
% 三维切向方向 τ（取代“固定顺时针”假设）：
%   n   = (x-x_obs)/‖x-x_obs‖（最近点法向）
%   e1  = F_att 的切平面投影单位向量（退化时取 b = n×ref 作为替代）
%   (n,e1) 张成决策平面；将 vr=p-c、vg=goal-c 投影到该平面得二维坐标
%   σ  = sign( a1*b2 - a2*b1 )（决策平面内的二维叉积判据）
%   τ  = σ · (n × e1)/‖n × e1‖
%   该定义在二维退化情形下与二维叉积判据一致，且对任意障碍物几何良定义。
%
% 安全与鲁棒性：候选位移必须通过精确碰撞检测（collisionChecking3D，可带
% 安全裕度）；被阻挡时尝试 ±30° 旋转、切向短步与方向翻转；网格访问计数
% 检测循环；停滞超限终止（可回退 RRT*）。
%
% info 字段：success, fallback, iterations, length, escapes, sideFlips,
%            cycles, timeLocal, firstSolutionTime。

if nargin < 2, opts = struct; end
def = struct('step', 3, 'rho0', 15, 'kAtt', 1, 'kRep', 1.1, 'kTan', 0.8, ...
    'maxSteps', 2500, 'goalTol', 4, 'balanceTol', 0.15, ...
    'cycleLimit', 3, 'sideLimit', 12, 'stuckLimit', 40, 'safetyMargin', 0, ...
    'progressLimit', 60, 'escapeBias', 0.5, 'wideSearch', 1, 'wideK', 16, ...
    'wideProbe', 2.0, 'wideBisect', 10, 'wideLambda', 0.6, 'wideMinStep', 0.02, ...
    'progressTol', 0, 'wideMode', 1, 'wideVisitMu', 0);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end

p = env.start; path = p;
info = struct('success', false, 'fallback', false, 'iterations', 0, 'length', inf, ...
    'escapes', 0, 'sideFlips', 0, 'cycles', 0, 'wideSteps', 0, 'timeLocal', inf, 'firstSolutionTime', NaN);
t0 = tic;

b = env.bounds;
cellSize = max(2 * opts.step, opts.rho0 / 2);
nx = max(1, ceil((b(2) - b(1)) / cellSize));
ny = max(1, ceil((b(4) - b(3)) / cellSize));
nz = max(1, ceil((b(6) - b(5)) / cellSize));
visits = zeros(nx, ny, nz);

side = 0; prevMode = 1; stagn = 0; lastFlip = [NaN NaN NaN];
bestDist = inf; noProgress = 0;

% 起终点直连快速判定
if logical(collisionChecking3D(env.start, env.goal, env, opts.safetyMargin))
    path = [env.start; env.goal];
    info.success = true; info.iterations = 1; info.length = norm(env.goal - env.start);
    info.timeLocal = toc(t0); info.firstSolutionTime = info.timeLocal;
    return;
end

for it = 1:opts.maxSteps
    dg = norm(env.goal - p);
    % progressTol > 0 时，只有“实质推进”（距离减少超过阈值）才重置停滞计数；
    % 原实现用 1e-9，原地绕圈时偶发的微小改善也会把计数清零，导致逃逸／广域搜索
    % 永远不触发（三维悬挑场景就是这样一直小幅挪动而耗尽预算）。
    if dg < bestDist - max(1e-9, opts.progressTol)
        bestDist = dg; noProgress = 0;
    else
        noProgress = noProgress + 1;
    end
    losFree = logical(collisionChecking3D(p, env.goal, env, opts.safetyMargin));
    if dg <= opts.goalTol && losFree
        path(end + 1, :) = env.goal; %#ok<AGROW>
        info.success = true;
        info.firstSolutionTime = toc(t0);
        break;
    end

    fatt = (env.goal - p) / max(dg, eps);
    [dnear, nrm, oid] = obstacle_query3d(p, env);
    frep = [0 0 0]; ftan = [0 0 0]; tdir = [0 0 0];
    balanced = false;
    if ~losFree && dnear < opts.rho0 && dnear > 1e-9
        w = min((1 / dnear - 1 / opts.rho0) / dnear^2, 20);
        frep = opts.kRep * w * nrm;
        fproj = abs(dot(fatt, nrm));
        fremag = norm(frep);
        balanced = abs(fproj - fremag) <= max(opts.balanceTol * fremag, 1e-6);
        if side == 0
            [side, ~] = circumvention_3d(p, env.goal, nrm, oid, env);
        end
        [~, tdir] = circumvention_3d(p, env.goal, nrm, oid, env, side);
        ftan = opts.kTan * max(1 - dnear / opts.rho0, 0.2) * tdir;
    end

    if losFree
        mode = 1; F = opts.kAtt * fatt;
        stepLen = opts.step * min(1 + dnear / opts.rho0, 2);
        % 视线可直达目标时不得越过目标（否则在目标附近以 2Δs 幅度来回过冲，
        % 实测会在两个点之间无限循环、耗尽步数预算）。
        stepLen = max(min(stepLen, dg), 1e-9);
    elseif (balanced && dnear < 0.6 * opts.rho0) || ...
            (noProgress > opts.progressLimit && dnear > 1e-9 && ~losFree)
        % 逃逸状态：沿切向绕行，并加入目标偏置以避免原地绕圈
        % （触发条件不要求障碍物进入影响半径，因为远距离遮挡同样造成局部极小）
        mode = 3;
        if norm(tdir) > 1e-9
            F = tdir / norm(tdir) + opts.escapeBias * fatt;
        else
            F = opts.escapeBias * fatt;
        end
        stepLen = opts.step;
    else
        mode = 2;
        beta = min(max((opts.rho0 - dnear) / opts.rho0, 0), 1);
        F = opts.kAtt * fatt + beta * (frep + ftan);
        stepLen = opts.step;
    end
    if mode == 3 && prevMode ~= 3, info.escapes = info.escapes + 1; end
    prevMode = mode;

    if norm(F) < 1e-9, F = fatt; end
    dirF = F / norm(F);

    q = []; accepted = false;
    % wideMode >= 2：逃逸状态（进度停滞 / 平衡力判据）也优先用广域搜索选位移。
    if opts.wideSearch > 0 && opts.wideMode >= 2 && mode == 3
        qw = wide_step3(p, env, opts, fatt, nrm, tdir, stepLen, widegrid(visits, cellSize, b));
        if ~isempty(qw)
            q = qw; accepted = true; info.wideSteps = info.wideSteps + 1;
        end
    end
    % 候选方向集合：合力方向，以及绕参考法向 ±30°/±60°/±90° 的旋转；
    % 若最近障碍在影响半径内，再并入切向绕行方向（含小角度偏转）。
    if dnear < opts.rho0 && dnear > 1e-9
        nref = nrm; tdirU = tdir / max(norm(tdir), eps);
    else
        nref = fatt; tdirU = [];
    end
    trials = {dirF};
    for ang = [pi / 6, -pi / 6, pi / 3, -pi / 3, pi / 2, -pi / 2]
        trials{end + 1} = rot3(dirF, nref, ang); %#ok<AGROW>
    end
    if ~isempty(tdirU)
        trials{end + 1} = tdirU; %#ok<AGROW>
        trials{end + 1} = rot3(tdirU, nref, pi / 4); %#ok<AGROW>
        trials{end + 1} = rot3(tdirU, nref, -pi / 4); %#ok<AGROW>
    end
    for fr = [1, 0.5]
        if accepted, break; end        % 逃逸状态的广域搜索已给出位移，不被标准候选覆盖
        for zz = 1:numel(trials)
            qq = p + fr * stepLen * trials{zz};
            if inside3(qq, env) && logical(collisionChecking3D(p, qq, env, opts.safetyMargin))
                q = qq; accepted = true; break;
            end
        end
        if accepted, break; end
    end
    % 广域搜索：标准候选集全部受阻时，在决策平面内枚举 K 个方向 + 最近障碍法向
    % 的正负向，各自折半求最大无碰撞推进长度，取 argmax(ℓ + λ·Δs·cos∠(d, 目标))。
    if ~accepted && opts.wideSearch > 0 && ~losFree
        qw = wide_step3(p, env, opts, fatt, nrm, tdir, stepLen, widegrid(visits, cellSize, b));
        if ~isempty(qw)
            q = qw; accepted = true; info.wideSteps = info.wideSteps + 1;
        end
    end

    % 全部候选受阻：翻转绕行方向，但同一位置不连续翻面（滞回），避免拉锅式抖动
    if ~accepted && dnear < opts.rho0 && dnear > 1e-9
        if side == 0
            [side, ~] = circumvention_3d(p, env.goal, nrm, oid, env);
        end
        if isnan(lastFlip(1)) || norm(p - lastFlip) > opts.step
            side = -side; lastFlip = p; info.sideFlips = info.sideFlips + 1;
        end
        if info.sideFlips > opts.sideLimit, break; end
        [~, tt] = circumvention_3d(p, env.goal, nrm, oid, env, side);
        for fr = [0.5, 0.25]
            qq = p + fr * stepLen * tt;
            if inside3(qq, env) && logical(collisionChecking3D(p, qq, env, opts.safetyMargin))
                q = qq; accepted = true; break;
            end
        end
    end

    if accepted
        p = q; path(end + 1, :) = p; %#ok<AGROW>
        stagn = 0;
    else
        stagn = stagn + 1;
    end

    cx = min(max(floor((p(1) - b(1)) / cellSize) + 1, 1), nx);
    cy = min(max(floor((p(2) - b(3)) / cellSize) + 1, 1), ny);
    cz = min(max(floor((p(3) - b(5)) / cellSize) + 1, 1), nz);
    visits(cx, cy, cz) = visits(cx, cy, cz) + 1;
    if visits(cx, cy, cz) == opts.cycleLimit
        info.cycles = info.cycles + 1;
        side = -side; info.sideFlips = info.sideFlips + 1;
        if info.sideFlips > opts.sideLimit, break; end
    end
    if stagn > opts.stuckLimit, break; end
end

% 版本说明（2026-09-18）：回退链已删除。本规划器不再调用采样型规划器，失败时
% 直接返回 success=false；上层在无种子下退化为 Informed-RRT* 冷启动采样。
if ~info.success
    info.fallback = false;      % 保留字段供上层记录，恒为 false
end

info.iterations = size(path, 1) - 1;
info.timeLocal = toc(t0);
if info.success
    info.length = path_length3(path);
else
    info.length = inf;
end
end

function g = widegrid(visits, cellSize, b)
%WIDEGRID 打包循环检测网格参数，供 wide_step3 计算“回访惩罚”。
g = struct('visits', visits, 'cell', cellSize, 'b', b);
end

function q = wide_step3(p, env, opts, gdir, nrm, tdir, stepLen, grd)
%WIDE_STEP3 最宽方向局部搜索（三维）：返回一步位移，无可行方向时返回 []。
% 方向集合 = {目标方向} ∪ {决策平面内 K 个均匀方向} ∪ {±最近障碍法向}。
q = [];
if norm(tdir) > 1e-9
    e1 = tdir / norm(tdir);
else
    g = gdir - dot(gdir, nrm) * nrm;
    if norm(g) < 1e-9, g = [0 0 1] - dot([0 0 1], nrm) * nrm; end
    if norm(g) < 1e-9, g = [1 0 0] - dot([1 0 0], nrm) * nrm; end
    e1 = g / max(norm(g), eps);
end
e2 = cross(nrm, e1);
if norm(e2) < 1e-9
    ref = [0 0 1]; if abs(dot(ref, nrm)) > 0.9, ref = [1 0 0]; end
    e2 = cross(nrm, ref);
end
e2 = e2 / max(norm(e2), eps);
dirs = cell(1, opts.wideK + 3);
dirs{1} = gdir;
for k = 0:opts.wideK - 1
    th = 2 * pi * k / opts.wideK;
    dirs{k + 2} = cos(th) * e1 + sin(th) * e2;
end
dirs{opts.wideK + 2} = nrm;
dirs{opts.wideK + 3} = -nrm;
probe = opts.wideProbe * stepLen;
lam = opts.wideLambda * stepLen;
bestScore = -inf; bestDir = []; bestLen = 0;
for k = 1:numel(dirs)
    dk = dirs{k} / max(norm(dirs{k}), eps);
    lo = 0; hi = probe;
    for it = 1:opts.wideBisect
        mid = 0.5 * (lo + hi);
        if inside3(p + mid * dk, env) && logical(collisionChecking3D(p, p + mid * dk, env, opts.safetyMargin))
            lo = mid;
        else
            hi = mid;
        end
    end
    if lo < opts.wideMinStep * stepLen, continue; end
    s = lo + lam * dot(dk, gdir);
    if opts.wideVisitMu > 0 && ~isempty(grd)
        pv = p + min(lo, stepLen) * dk;
        v = grd.visits;
        cx = min(max(floor((pv(1) - grd.b(1)) / grd.cell) + 1, 1), size(v, 1));
        cy = min(max(floor((pv(2) - grd.b(3)) / grd.cell) + 1, 1), size(v, 2));
        cz = min(max(floor((pv(3) - grd.b(5)) / grd.cell) + 1, 1), size(v, 3));
        s = s - opts.wideVisitMu * stepLen * v(cx, cy, cz);
    end
    if s > bestScore
        bestScore = s; bestDir = dk; bestLen = min(lo, stepLen);
    end
end
if ~isempty(bestDir)
    q = p + bestLen * bestDir;
end
end

function [side, t] = circumvention_3d(p, goal, n, oid, env, sideIn)
% 三维绕行方向判定（见文件头公式）。
c = obstacle_center3d(env, oid);
g = goal - p; gp = g - dot(g, n) * n;
if norm(gp) < 1e-9
    ref = [0 0 1];
    if abs(dot(ref, n)) > 0.9, ref = [1 0 0]; end
    e1 = cross(n, ref); e1 = e1 / max(norm(e1), eps);
else
    e1 = gp / norm(gp);
end
vr = p - c; vg = goal - c;
a = [dot(vr, n), dot(vr, e1)];
b2 = [dot(vg, n), dot(vg, e1)];
cr = a(1) * b2(2) - a(2) * b2(1);
if nargin >= 6 && ~isempty(sideIn)
    side = sideIn;
else
    if cr >= 0, side = 1; else, side = -1; end
end
tt = cross(n, e1);
if norm(tt) < 1e-9
    ref = [0 0 1];
    if abs(dot(ref, n)) > 0.9, ref = [1 0 0]; end
    tt = cross(n, ref);
end
t = side * tt / max(norm(tt), eps);
end

function c = obstacle_center3d(env, oid)
% 与 obstacle_query3d 的编号约定一致：1..999 立方体、2000+k 圆柱、3000+k 球。
if oid == 0
    c = env.bounds(1:2:6); return;
end
if oid < 1000
    c = [env.cube.axisX(oid) + env.cube.lengthx(oid) / 2, ...
         env.cube.axisY(oid) + env.cube.lengthy(oid) / 2, ...
         env.cube.axisZ(oid) + env.cube.lengthz(oid) / 2];
elseif oid < 3000
    k = oid - 2000;
    c = [env.cylinder.X(k), env.cylinder.Y(k), env.cylinder.Z(k) + env.cylinder.lengthZ(k) / 2];
else
    k = oid - 3000;
    c = [env.sphere.X(k), env.sphere.Y(k), env.sphere.Z(k)];
end
end

function v = rot3(v, n, a)
% 将向量 v 绕轴 n 旋转角度 a（用于被阻挡时的试探方向）。
n = n / max(norm(n), eps);
v = v * cos(a) + cross(n, v) * sin(a) + n * dot(n, v) * (1 - cos(a));
end

function ok = inside3(p, env)
b = env.bounds;
ok = p(1) >= b(1) && p(1) <= b(2) && p(2) >= b(3) && p(2) <= b(4) && ...
     p(3) >= b(5) && p(3) <= b(6);
end

function L = path_length3(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
