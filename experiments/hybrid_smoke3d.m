function hybrid_smoke3d()
%HYBRID_SMOKE3D 新增混合基线的冒烟测试（三维）。
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(fullfile(rootDir, 'common')); addpath(fullfile(rootDir, 'common3d'));

modes = {'general', 'narrow', 'suspended', 'ring', 'overhang'};
algos = {'APF-IRRT*', 'HAS-RRT', 'AB-IRRT*'};
common = struct('maxIter', 6000, 'step', 6, 'goalBias', 0.12, 'rewireRadius', 18, ...
    'safetyMargin', 0);

fprintf('%-10s %-12s %-4s %9s %9s %9s %6s %7s\n', 'env', 'algorithm', 'ok', 'length', 'time_s', 'firstTim', 'nodes', 'badSeg');
log = {};
log{end + 1} = sprintf('%-10s %-12s %-4s %9s %9s %9s %6s %7s', 'env', 'algorithm', 'ok', ...
    'length', 'time_s', 'firstTim', 'nodes', 'badSeg');
for c = 1:numel(modes)
    env = bair_env3d(modes{c});
    for a = 1:numel(algos)
        rng(20260903 + 1000 * c + 1, 'twister');
        t0 = tic;
        switch algos{a}
            case 'APF-IRRT*'
                [path, info] = apf_irrtstar3d(env, common);
            case 'HAS-RRT'
                [path, info] = has_rrt3d(env, common);
            case 'AB-IRRT*'
                bo = common; bo.pbias = 0.1; bo.dynamicRewire = true; bo.rewireMax = 30;
                bo.useFallback = true;
                [~, ~, detail] = bair_core3d(modes{c}, bo);
                path = detail.path; info = detail.rrt;
        end
        tt = toc(t0);
        bad = bad_segments3(path, env);
        L = NaN; if size(path, 1) >= 2, L = sum(sqrt(sum(diff(path).^2, 2))); end
        log{end + 1} = sprintf('%-10s %-12s %-4d %9.2f %9.3f %9.4f %6d %7d', modes{c}, ...
            algos{a}, info.success, L, tt, info.firstSolutionTime, info.nodes, bad);
        fprintf('%s\n', log{end});
        if isfield(info, 'skeletonOK')
            log{end + 1} = sprintf('    skeleton ok=%d verts=%d tBuild=%.3f regions=%d', ...
                info.skeletonOK, info.skeletonVerts, info.skeletonTime, info.regions);
            fprintf('%s\n', log{end});
        end
    end
end
log{end + 1} = 'smoke3d done';
fid = fopen(fullfile(entryDir, '_smoke3d.txt'), 'w', 'n', 'UTF-8');
fprintf(fid, '%s\n', log{:});
fclose(fid);
fprintf('smoke3d done\n');
end

function nbad = bad_segments3(path, env)
nbad = 0;
if size(path, 1) < 2, return; end
for i = 1:size(path, 1) - 1
    if ~collisionChecking3D(path(i, :), path(i + 1, :), env), nbad = nbad + 1; end
end
end
