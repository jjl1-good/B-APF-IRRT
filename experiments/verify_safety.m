function report = verify_safety(nRuns, opts)
%VERIFY_SAFETY 独立安全性复检：重跑各算法并逐段验证返回路径是否无碰撞。
%
%   report = verify_safety(20)
%
% 复检内容：
%   1) 返回路径的每一段是否通过精确碰撞检测（collisionChecking / 3D 版本）；
%   2) 路径是否有限长、是否含 NaN；
%   3) 统计发生碰撞的线段总数与出现碰撞的运行次数。
% 输出 CSV：verify_safety_2d.csv、verify_safety_3d.csv（UTF-8）。
%
% 该脚本用于支撑 5.9 节“局部阶段未输出碰撞位移”的论断，结果与主实验独立。

if nargin < 1 || isempty(nRuns), nRuns = 20; end
if nargin < 2, opts = struct; end
def = struct('maxIter2d', 2000, 'maxIter3d', 3000, 'quiet', false);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));

algs2d = {'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only', 'AB-IRRT*'};
scenes2d = {'一般', '狭窄', '广阔'};
modes2d = {'y', 'n', 'g'};
scenes3d = {'一般', '环廊'};
modes3d = {'general_v2', 'ring'};
algs3d = {'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only', 'AB-IRRT*'};

rows = struct('dimension', {}, 'scene', {}, 'algorithm', {}, 'runs', {}, 'successRuns', {}, ...
    'invalidPaths', {}, 'collidingSegments', {}, 'notes', {});
z = 0;
baseSeed = 20260903;

for c = 1:numel(modes2d)
    env = bair_env2d(modes2d{c});
    isFree = @(a, b) collisionChecking(a, b, env.squareAll, env.round);
    for a = 1:numel(algs2d)
        invPath = 0; invSeg = 0; succ = 0; notes = '';
        for r = 1:nRuns
            rng(baseSeed + 1000 * c + r, 'twister');
            [p, ok] = run_alg2d(algs2d{a}, env, isFree, opts.maxIter2d);
            if ~ok, continue; end
            if isempty(p) || size(p, 1) < 2, continue; end
            succ = succ + 1;
            [bad, nseg, note] = check_path2d(p, isFree, env);
            if bad, invPath = invPath + 1; invSeg = invSeg + nseg; end
            if ~isempty(note), notes = [notes, ' ', note]; end %#ok<AGROW>
        end
        z = z + 1;
        rows(z) = struct('dimension', '2D', 'scene', scenes2d{c}, 'algorithm', algs2d{a}, ...
            'runs', nRuns, 'successRuns', succ, 'invalidPaths', invPath, ...
            'collidingSegments', invSeg, 'notes', strtrim(notes));
        if ~opts.quiet
            fprintf('2D %s %-16s: success %d/%d, invalid paths %d, colliding segments %d\n', ...
                scenes2d{c}, algs2d{a}, succ, nRuns, invPath, invSeg);
        end
    end
end

for c = 1:numel(modes3d)
    env = bair_env3d(modes3d{c});
    isFree = @(a, b) collisionChecking3D(a, b, env);
    for a = 1:numel(algs3d)
        invPath = 0; invSeg = 0; succ = 0; notes = '';
        for r = 1:nRuns
            rng(baseSeed + 1000 * c + r, 'twister');
            [p, ok] = run_alg3d(algs3d{a}, env, isFree, opts.maxIter3d, modes3d{c});
            if ~ok, continue; end
            if isempty(p) || size(p, 1) < 2, continue; end
            succ = succ + 1;
            [bad, nseg, note] = check_path3d(p, isFree, env);
            if bad, invPath = invPath + 1; invSeg = invSeg + nseg; end
            if ~isempty(note), notes = [notes, ' ', note]; end %#ok<AGROW>
        end
        z = z + 1;
        rows(z) = struct('dimension', '3D', 'scene', scenes3d{c}, 'algorithm', algs3d{a}, ...
            'runs', nRuns, 'successRuns', succ, 'invalidPaths', invPath, ...
            'collidingSegments', invSeg, 'notes', strtrim(notes));
        if ~opts.quiet
            fprintf('3D %s %-16s: success %d/%d, invalid paths %d, colliding segments %d\n', ...
                scenes3d{c}, algs3d{a}, succ, nRuns, invPath, invSeg);
        end
    end
end

report = struct2table(rows);
writetable(report, fullfile(entryDir, 'verify_safety.csv'), 'Encoding', 'UTF-8');
fprintf('safety verification written to verify_safety.csv\n');
end

function [p, ok] = run_alg2d(alg, env, isFree, maxIter)
o = struct('maxIter', maxIter, 'step', 4, 'goalBias', 0.08, 'rewireRadius', 14);
ok = false;
switch alg
    case 'RRT*', [p, i] = rrtstar2d(env, o); ok = i.success;
    case 'Informed-RRT*', [p, i] = informed_rrtstar2d(env, [], o); ok = i.success;
    case 'RRT-Connect', [p, i] = rrtconnect2d(env, o); ok = i.success;
    case 'PRM*', po = o; po.maxNodes = maxIter; po.gamma = 1.0; po.kMax = 12; [p, i] = prmstar_nd(env.bounds, env.start, env.goal, isFree, po); ok = i.success;
    case 'FMT*', po = o; po.maxNodes = maxIter; po.gamma = 0.8; [p, i] = fmtstar_nd(env.bounds, env.start, env.goal, isFree, po); ok = i.success;
    case 'BIT*', po = o; po.maxNodes = maxIter; po.gamma = 0.9; [p, i] = bitstar_nd(env.bounds, env.start, env.goal, isFree, po); ok = i.success;
    case 'RRT#', [p, i] = rrtsharp_nd(env.bounds, env.start, env.goal, isFree, o); ok = i.success;
    case 'APF-only', ao = struct('useFallback', false, 'step', o.step); [p, i] = stable_apf2d(env, ao); ok = i.success;
    case 'AB-IRRT*', bo = o; bo.pbias = 0.5; bo.dynamicRewire = true; bo.rewireMax = 16; bo.useFallback = true; [~, ~, d] = bair_core2d(env.name, bo); p = d.path; ok = d.success;
    otherwise, p = zeros(0, 2);
end
end

function [p, ok] = run_alg3d(alg, env, isFree, maxIter, mode)
o = struct('maxIter', maxIter, 'step', 6, 'goalBias', 0.08, 'rewireRadius', 18);
ok = false;
switch alg
    case 'RRT*', [p, i] = rrtstar3d(env, o); ok = i.success;
    case 'Informed-RRT*', [p, i] = informed_rrtstar3d(env, [], o); ok = i.success;
    case 'RRT-Connect', [p, i] = rrtconnect3d(env, o); ok = i.success;
    case 'PRM*', po = o; po.maxNodes = maxIter; po.gamma = 1.0; po.kMax = 12; [p, i] = prmstar_nd(env.bounds, env.start, env.goal, isFree, po); ok = i.success;
    case 'FMT*', po = o; po.maxNodes = maxIter; po.gamma = 0.7; [p, i] = fmtstar_nd(env.bounds, env.start, env.goal, isFree, po); ok = i.success;
    case 'BIT*', po = o; po.maxNodes = maxIter; po.gamma = 0.65; [p, i] = bitstar_nd(env.bounds, env.start, env.goal, isFree, po); ok = i.success;
    case 'RRT#', [p, i] = rrtsharp_nd(env.bounds, env.start, env.goal, isFree, o); ok = i.success;
    case 'APF-only', ao = struct('useFallback', false, 'step', o.step); [p, i] = stable_apf3d(env, ao); ok = i.success;
    case 'AB-IRRT*', bo = o; bo.pbias = 0.1; bo.dynamicRewire = true; bo.rewireMax = 30; bo.useFallback = true; [~, ~, d] = bair_core3d(mode, bo); p = d.path; ok = d.success;
    otherwise, p = zeros(0, 3);
end
end

function [bad, nBad, note] = check_path2d(p, isFree, env)
% 逐段碰撞复检 + 起终点连通性检查（note 记录异常类型）。
bad = false; nBad = 0; note = '';
if any(~isfinite(p(:)))
    bad = true; nBad = nBad + 1; note = 'non-finite'; return;
end
if norm(p(1, :) - env.start(:)') > 1e-6
    bad = true; note = 'start-not-connected';
end
if norm(p(end, :) - env.goal(:)') > 1e-6
    bad = true; note = [note, ' goal-not-connected'];
end
for i = 1:size(p, 1) - 1
    if ~logical(isFree(p(i, :), p(i + 1, :)))
        bad = true; nBad = nBad + 1;
    end
end
end

function [bad, nBad, note] = check_path3d(p, isFree, env)
bad = false; nBad = 0; note = '';
if any(~isfinite(p(:)))
    bad = true; nBad = nBad + 1; note = 'non-finite'; return;
end
if norm(p(1, :) - env.start(:)') > 1e-6
    bad = true; note = 'start-not-connected';
end
if norm(p(end, :) - env.goal(:)') > 1e-6
    bad = true; note = [note, ' goal-not-connected'];
end
for i = 1:size(p, 1) - 1
    if ~logical(isFree(p(i, :), p(i + 1, :)))
        bad = true; nBad = nBad + 1;
    end
end
end
