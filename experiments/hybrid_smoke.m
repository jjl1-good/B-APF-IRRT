function hybrid_smoke()
%HYBRID_SMOKE 新增文献混合基线（APF-IRRT* / HAS-RRT）的冒烟测试，2D+3D。
%   校验：路径成功、逐段无碰撞，并记录骨架/区域规模与耗时。
%   结果写入 _smoke.txt（UTF-8），避免终端编码问题。
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(fullfile(rootDir, 'common')); addpath(fullfile(rootDir, 'common3d'));

L = {};
c2 = struct('maxIter', 2000, 'step', 4, 'goalBias', 0.08, 'rewireRadius', 14, 'safetyMargin', 0);
modes2 = {'y', 'n', 'g'};
names2 = {'2D-一般', '2D-狭窄', '2D-广阔'};
algos2 = {'APF-IRRT*', 'HAS-RRT', 'Warm-Informed-RRT*', 'AB-IRRT*'};
L{end + 1} = '== 2D ==';
for c = 1:numel(modes2)
    env = bair_env2d(modes2{c});
    for a = 1:numel(algos2)
        rng(20260903 + 1000 * c + 1, 'twister');
        t0 = tic;
        switch algos2{a}
            case 'APF-IRRT*'
                [path, info] = apf_irrtstar2d(env, c2);
            case 'HAS-RRT'
                [path, info] = has_rrt2d(env, c2);
            case 'Warm-Informed-RRT*'
                wo = c2; wo.useFallback = true;
                [sp, ~] = stable_apf2d(env, wo);
                io = c2; io.pbias = 0.0; io.dynamicRewire = false; io.useFallback = true;
                [path, info] = informed_rrtstar2d(env, sp, io);
            case 'AB-IRRT*'
                bo = c2; bo.pbias = 0.5; bo.dynamicRewire = true; bo.rewireMax = 16;
                [~, ~, detail] = bair_core2d(modes2{c}, bo);
                path = detail.path; info = detail.rrt;
        end
        tt = toc(t0);
        L{end + 1} = sprintf('%-8s %-20s ok=%d L=%8.2f t=%7.3f firstT=%8.4f nodes=%5d bad=%d', ...
            names2{c}, algos2{a}, info.success, len(path), tt, info.firstSolutionTime, ...
            info.nodes, bad2(path, env));
        if isfield(info, 'skeletonOK')
            L{end + 1} = sprintf('        skeleton ok=%d verts=%d tBuild=%.3f regions=%d', ...
                info.skeletonOK, info.skeletonVerts, info.skeletonTime, info.regions);
        end
    end
end

c3 = struct('maxIter', 6000, 'step', 6, 'goalBias', 0.12, 'rewireRadius', 18, 'safetyMargin', 0);
modes3 = {'general', 'narrow', 'suspended', 'ring', 'overhang'};
algos3 = {'APF-IRRT*', 'HAS-RRT', 'AB-IRRT*'};
L{end + 1} = '== 3D ==';
for c = 1:numel(modes3)
    env = bair_env3d(modes3{c});
    for a = 1:numel(algos3)
        rng(20260903 + 1000 * c + 1, 'twister');
        t0 = tic;
        switch algos3{a}
            case 'APF-IRRT*'
                [path, info] = apf_irrtstar3d(env, c3);
            case 'HAS-RRT'
                [path, info] = has_rrt3d(env, c3);
            case 'AB-IRRT*'
                bo = c3; bo.pbias = 0.1; bo.dynamicRewire = true; bo.rewireMax = 30;
                bo.useFallback = true;
                [~, ~, detail] = bair_core3d(modes3{c}, bo);
                path = detail.path; info = detail.rrt;
        end
        tt = toc(t0);
        L{end + 1} = sprintf('%-10s %-20s ok=%d L=%8.2f t=%7.3f firstT=%8.4f nodes=%5d bad=%d', ...
            modes3{c}, algos3{a}, info.success, len(path), tt, info.firstSolutionTime, ...
            info.nodes, bad3(path, env));
        if isfield(info, 'skeletonOK')
            L{end + 1} = sprintf('        skeleton ok=%d verts=%d tBuild=%.3f regions=%d', ...
                info.skeletonOK, info.skeletonVerts, info.skeletonTime, info.regions);
        end
    end
end
L{end + 1} = 'smoke done';
fid = fopen(fullfile(entryDir, '_smoke.txt'), 'w', 'n', 'UTF-8');
fprintf(fid, '%s\n', L{:});
fclose(fid);
fprintf('smoke done\n');
end

function L = len(p)
if size(p, 1) < 2, L = NaN; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end

function n = bad2(p, env)
n = 0;
for i = 1:size(p, 1) - 1
    if ~collisionChecking(p(i, :), p(i + 1, :), env.squareAll, env.round), n = n + 1; end
end
end

function n = bad3(p, env)
n = 0;
for i = 1:size(p, 1) - 1
    if ~collisionChecking3D(p(i, :), p(i + 1, :), env), n = n + 1; end
end
end
