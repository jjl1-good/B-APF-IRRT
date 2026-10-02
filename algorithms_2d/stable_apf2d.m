function [path, info] = stable_apf2d(env, opts)
%STABLE_APF2D 二维 Bug-APF 预规划器（形式化状态机版本）。
%
% 与论文 2.4 节的对应关系（可执行定义）：
%   S1 DIRECT : I_direct=1（对目标视线无遮挡）→ 纯吸引场 + 自适应步长
%               Δx_{t+1}=Δx_t(1+d/ρ0)（上限 2Δx）
%   S2 AVOID  : 视线受阻且 d<ρ0 → F=F_att+β(F_rep+F_tex)，
%               β=(ρ0-d)/ρ0 为平滑混合系数
%   S3 ESCAPE : 平衡力检测 |proj_n(F_att)|≈||F_rep|| 且 d<0.6ρ0
%               → 纯切向逃逸方向（仍逐步执行精确碰撞检测）
% 绕行方向 σ 由几何相对位置叉积判据初始化，并在循环检测触发时翻转：
%   σ = sign( (p-c) × (goal-c) )，c 为最近障碍物中心。
% 安全机制：每个候选位移必须通过 collisionChecking（可带安全裕度）；
% 被阻挡时依次尝试 ±30° 旋转、切向短步、翻转绕行方向。
% 循环/停滞检测：位置网格访问计数 + 停滞计数，超限则翻转或终止。
%
% info 字段：success, fallback, iterations, length, escapes, sideFlips,
%            cycles, timeLocal, firstSolutionTime。

if nargin < 2, opts = struct; end
def = struct('step', 2.0, 'rho0', 12, 'kAtt', 1.0, 'kRep', 1.2, 'kTan', 0.9, ...
    'maxSteps', 2500, 'goalTol', 3, 'balanceTol', 0.15, ...
    'cycleLimit', 3, 'sideLimit', 8, 'stuckLimit', 25, 'safetyMargin', 0, ...
    'wideSearch', 1, 'wideK', 16, 'wideProbe', 2.0, 'wideBisect', 10, ...
    'wideLambda', 0.6, 'wideMinStep', 0.02);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end

p = env.start; path = p;
info = struct('success', false, 'fallback', false, 'iterations', 0, 'length', inf, ...
    'escapes', 0, 'sideFlips', 0, 'cycles', 0, 'wideSteps', 0, 'timeLocal', inf, 'firstSolutionTime', NaN);t0 = tic;

% 循环检测网格
b = env.bounds;
cellSize = max(2 * opts.step, opts.rho0 / 2);
nx = max(1, ceil((b(2) - b(1)) / cellSize));
ny = max(1, ceil((b(4) - b(3)) / cellSize));
visits = zeros(nx, ny);

side = 0;            % 绕行方向（0 表示尚未初始化）
prevMode = 1;
stagn = 0;

for it = 1:opts.maxSteps
    dg = norm(env.goal - p);
    losFree = logical(collisionChecking(p, env.goal, env.squareAll, env.round, opts.safetyMargin));
    if dg <= opts.goalTol && losFree
        path(end + 1, :) = env.goal; %#ok<AGROW>
        info.success = true;
        info.firstSolutionTime = toc(t0);
        break;
    end

    fatt = (env.goal - p) / max(dg, eps);
    [dnear, nrm, oid] = obstacle_query2d(p, env);
    frep = [0 0]; ftan = [0 0]; tdir = [0 0];
    balanced = false;
    if ~losFree && dnear < opts.rho0 && dnear > 1e-9
        w = min((1 / dnear - 1 / opts.rho0) / dnear^2, 20);
        frep = opts.kRep * w * nrm;
        fproj = abs(dot(fatt, nrm));
        fremag = norm(frep);
        balanced = abs(fproj - fremag) <= max(opts.balanceTol * fremag, 1e-6);
        if side == 0
            side = side_criterion_2d(p, env, oid, env.goal);
        end
        tdir = side * [-nrm(2), nrm(1)];
        ftan = opts.kTan * max(1 - dnear / opts.rho0, 0.2) * tdir;
    end

    if losFree
        mode = 1; F = opts.kAtt * fatt;
        stepLen = opts.step * min(1 + dnear / opts.rho0, 2);   % 自适应步长
        % 视线可直达目标时不得越过目标：否则在目标邻域会以 2Δs 的幅度来回过冲，
        % 永远落不进 goalTol。截断到剩余距离后，一步正好落在目标上。
        stepLen = max(min(stepLen, dg), 1e-9);
    elseif balanced && dnear < 0.6 * opts.rho0
        mode = 3;                                              % 逃逸：纯切向
        F = tdir / max(norm(tdir), eps);
        stepLen = opts.step;
    else
        mode = 2;                                              % 绕行：平滑混合
        beta = min(max((opts.rho0 - dnear) / opts.rho0, 0), 1);
        F = opts.kAtt * fatt + beta * (frep + ftan);
        stepLen = opts.step;
    end
    if mode == 3 && prevMode ~= 3, info.escapes = info.escapes + 1; end
    prevMode = mode;

    if norm(F) < 1e-9, F = fatt; end
    dirF = F / norm(F);

    % 候选位移：主方向 → ±30° 旋转；全部受阻时用广域搜索（最宽方向），
    % 最后才是切向短步与翻转方向。
    q = []; accepted = false;
    trials = {dirF, rot2(dirF, pi / 6), rot2(dirF, -pi / 6)};
    for zz = 1:numel(trials)
        qq = p + stepLen * trials{zz};
        if inside2(qq, env) && logical(collisionChecking(p, qq, env.squareAll, env.round, opts.safetyMargin))
            q = qq; accepted = true; break;
        end
    end
    if ~accepted && opts.wideSearch > 0 && ~losFree
        % 广域搜索：枚举 K 个方向，各自用折半法求最大无碰撞推进长度 ℓ_k，
        % 取 argmax(ℓ_k + λ·Δs·cos∠(d_k, 目标))。缝隙场景里只有“指向缝内”的方向 ℓ 大，
        % 因此能选到穿缝方向；开放场景里朝目标项决定，行为与固定候选集一致。
        qw = wide_step2(p, env, opts, fatt, stepLen);
        if ~isempty(qw)
            q = qw; accepted = true; info.wideSteps = info.wideSteps + 1;
        end
    end

    if ~accepted && dnear < opts.rho0 && dnear > 1e-9
        if side == 0, side = 1; end
        tt = side * [-nrm(2), nrm(1)];
        qq = p + 0.5 * stepLen * tt;
        if inside2(qq, env) && logical(collisionChecking(p, qq, env.squareAll, env.round, opts.safetyMargin))
            q = qq; accepted = true;
        else
            side = -side; info.sideFlips = info.sideFlips + 1;
            tt = side * [-nrm(2), nrm(1)];
            qq = p + 0.5 * stepLen * tt;
            if inside2(qq, env) && logical(collisionChecking(p, qq, env.squareAll, env.round, opts.safetyMargin))
                q = qq; accepted = true;
            end
        end
    end

    if accepted
        p = q; path(end + 1, :) = p; %#ok<AGROW>
        stagn = 0;
    else
        stagn = stagn + 1;
    end

    % 循环检测：同一网格重复访问过多则翻转绕行方向
    cx = min(max(floor((p(1) - b(1)) / cellSize) + 1, 1), nx);
    cy = min(max(floor((p(2) - b(3)) / cellSize) + 1, 1), ny);
    visits(cx, cy) = visits(cx, cy) + 1;
    if visits(cx, cy) == opts.cycleLimit
        info.cycles = info.cycles + 1;
        side = -side; info.sideFlips = info.sideFlips + 1;
        if info.sideFlips > opts.sideLimit, break; end
    end
    if stagn > opts.stuckLimit, break; end
end

% 版本说明（2026-09-18）：回退链已删除。本规划器不再调用任何采样型规划器，
% 失败时直接返回 success=false；上层（阶段二）在无种子下退化为标准
% Informed-RRT* 的冷启动采样。
if ~info.success
    info.fallback = false;      % 保留字段供上层记录，恒为 false
end

info.iterations = size(path, 1) - 1;
info.timeLocal = toc(t0);
if info.success
    info.length = path_length2d(path);
else
    info.length = inf;
end
end

function s = side_criterion_2d(p, env, oid, goal)
% 几何相对位置叉积判据：σ = sign( (p-c) × (goal-c) )。
c = obstacle_center2d(env, oid);
vr = p - c; vg = goal - c;
cr = vr(1) * vg(2) - vr(2) * vg(1);
if cr >= 0, s = 1; else, s = -1; end
end

function c = obstacle_center2d(env, oid)
if oid == 0
    c = env.bounds(1:2); return;
end
if oid < 1000
    r = env.squareAll(oid, :);
    c = [r(1) + r(3) / 2, r(2) + r(4) / 2];
else
    k = oid - 1000;
    c = env.round(k, 1:2);
end
end

function v = rot2(v, a)
v = [cos(a) * v(1) - sin(a) * v(2), sin(a) * v(1) + cos(a) * v(2)];
end

function q = wide_step2(p, env, opts, gdir, stepLen)
%WIDE_STEP2 最宽方向局部搜索：返回一步位移（长度 ≤ 名义步长），无可行方向时返回 []。
q = [];
probe = opts.wideProbe * stepLen;
lam = opts.wideLambda * stepLen;
bestScore = -inf; bestDir = []; bestLen = 0;
for k = 0:opts.wideK - 1
    th = 2 * pi * k / opts.wideK;
    dk = [cos(th), sin(th)];
    lo = 0; hi = probe;
    for it = 1:opts.wideBisect
        mid = 0.5 * (lo + hi);
        if inside2(p + mid * dk, env) && logical(collisionChecking(p, p + mid * dk, env.squareAll, env.round, opts.safetyMargin))
            lo = mid;
        else
            hi = mid;
        end
    end
    if lo < opts.wideMinStep * stepLen, continue; end
    s = lo + lam * dot(dk, gdir);
    if s > bestScore
        bestScore = s; bestDir = dk; bestLen = min(lo, stepLen);
    end
end
if ~isempty(bestDir)
    q = p + bestLen * bestDir;
end
end

function ok = inside2(p, env)
b = env.bounds;
ok = p(1) >= b(1) && p(1) <= b(2) && p(2) >= b(3) && p(2) <= b(4);
end

function L = path_length2d(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
