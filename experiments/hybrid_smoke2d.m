function hybrid_smoke2d()
%HYBRID_SMOKE2D 新增混合基线的冒烟测试（二维）。
%   校验：路径成功、逐段无碰撞、指标可算、耗时量级可接受。
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(fullfile(rootDir, 'common')); addpath(fullfile(rootDir, 'common3d'));

modes = {'y', 'n', 'g'};
names = {'一般', '狭窄', '广阔'};
algos = {'APF-IRRT*', 'HAS-RRT', 'Warm-Informed-RRT*', 'AB-IRRT*'};
common = struct('maxIter', 2000, 'step', 4, 'goalBias', 0.08, 'rewireRadius', 14, ...
    'safetyMargin', 0);

fprintf('%-8s %-20s %-6s %8s %9s %9s\n', 'env', 'algorithm', 'ok', 'length', 'time_s', 'firstTime');
for c = 1:numel(modes)
    env = bair_env2d(modes{c});
    for a = 1:numel(algos)
        rng(20260903 + 1000 * c + 1, 'twister');
        t0 = tic;
        switch algos{a}
            case 'APF-IRRT*'
                [path, info] = apf_irrtstar2d(env, common);
            case 'HAS-RRT'
                [path, info] = has_rrt2d(env, common);
            case 'Warm-Informed-RRT*'
                wo = common; wo.useFallback = true;
                [seedPath, ~] = stable_apf2d(env, wo);
                io = common; io.pbias = 0.0; io.dynamicRewire = false; io.useFallback = true;
                [path, info] = informed_rrtstar2d(env, seedPath, io);
            case 'AB-IRRT*'
                bo = common; bo.pbias = 0.5; bo.dynamicRewire = true; bo.rewireMax = 16;
                [~, ~, detail] = bair_core2d(modes{c}, bo);
                path = detail.path; info = detail.rrt;
        end
        tt = toc(t0);
        bad = bad_segments(path, env);
        L = NaN; if size(path, 1) >= 2, L = sum(sqrt(sum(diff(path).^2, 2))); end
        fprintf('%-8s %-20s %-6d %8.2f %9.3f %9.4f  badSeg=%d\n', names{c}, algos{a}, ...
            info.success, L, tt, info.firstSolutionTime, bad);
    end
end
fprintf('smoke2d done\n');
end

function nbad = bad_segments(path, env)
nbad = 0;
if size(path, 1) < 2, return; end
for i = 1:size(path, 1) - 1
    if ~collisionChecking(path(i, :), path(i + 1, :), env.squareAll, env.round)
        nbad = nbad + 1;
    end
end
end
