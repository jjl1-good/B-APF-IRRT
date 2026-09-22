function smoke_test()
%SMOKE_TEST  Installation check: are all algorithms, environments and the collision
%   checker on the path, and do they run?
%
%   First every function needed by the paper is looked up on the MATLAB path
%   (2-D and 3-D planners, environments, collision checking), then all eight
%   algorithms are run once in the 2-D general map with the settings and the
%   seed used in the paper, and every returned path is checked segment by
%   segment.
%
%   Takes a few seconds and writes no file. If this passes, the clone is complete
%   and RUN_2D_COMPARISON / RUN_3D_COMPARISON can be started.
%
%   See also RUN_2D_COMPARISON, RUN_3D_COMPARISON, RUN_ALL_COMPARISONS.

entryDir = fileparts(mfilename('fullpath'));
rootDir  = fileparts(entryDir);
addpath(entryDir);
addpath(fullfile(rootDir, 'algorithms_2d'));
addpath(fullfile(rootDir, 'algorithms_3d'));

fprintf('smoke test: %s\n', rootDir);

% ---- 1) all functions present on the path? -------------------------------
need = {'bair_env2d', 'bair_core2d', 'stable_apf2d', 'stable_local_planner2d', ...
        'rrtstar2d', 'informed_rrtstar2d', 'prmstar_nd', 'fmtstar_nd', ...
        'bitstar_nd', 'rrtsharp_nd', 'collisionChecking', ...
        'bair_env3d', 'bair_core3d', 'collisionChecking3D'};
missing = {};
for i = 1:numel(need)
    if exist(need{i}, 'file') ~= 2
        missing{end + 1} = need{i};                          %#ok<AGROW>
    end
end
fprintf('  functions found: %d/%d\n', numel(need) - numel(missing), numel(need));
if ~isempty(missing)
    error('smoke_test:missing', 'missing functions: %s', strjoin(missing, ', '));
end

% ---- 2) run all eight algorithms once in the 2-D general map -------------
mode = 'y';
env  = bair_env2d(mode);
isFree = @(a, b) collisionChecking(a, b, env.squareAll, env.round);
common = struct('maxIter', 2000, 'step', 4, 'goalBias', 0.08, 'rewireRadius', 14);
seed = 20260903;
names = {'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only', 'AB-IRRT*'};

% ---- 2) run all eight algorithms once in the 2-D general map -------------
mode = 'y';
env  = bair_env2d(mode);
isFree = @(a, b) collisionChecking(a, b, env.squareAll, env.round);
common = struct('maxIter', 2000, 'step', 4, 'goalBias', 0.08, 'rewireRadius', 14);
seed = 20260903;
names = {'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only', 'AB-IRRT*'};

fprintf('  %-16s %6s %12s %10s %8s\n', 'algorithm', 'ok', 'length', 'time [ms]', 'collision');
free = false(1, numel(names));
tAll = tic;
for i = 1:numel(names)
    rng(seed, 'twister');               % same seed for every algorithm, as in the paper
    t0 = tic;
    [p, ok] = run_alg(names{i}, env, isFree, common, mode);
    ms = toc(t0) * 1000;
    free(i) = report(names{i}, p, ok, ms, env);
end

if all(free)
    fprintf('  OK: all %d algorithms returned a collision-free path in %.1f s total.\n', ...
        numel(names), toc(tAll));
end
end


% -------------------------------------------------------------------------
function [path, ok] = run_alg(name, env, isFree, common, mode)
%RUN_ALG  One algorithm, called exactly as the experiment scripts call it.
info = struct();
ok = true;
switch name
    case 'RRT*'
        [path, info] = rrtstar2d(env, common);
    case 'Informed-RRT*'
        [path, info] = informed_rrtstar2d(env, [], common);
    case 'PRM*'
        po = common;  po.maxNodes = common.maxIter;  po.gamma = 1.0;  po.kMax = 12;
        [path, info] = prmstar_nd(env.bounds, env.start, env.goal, isFree, po);
    case 'FMT*'
        po = common;  po.maxNodes = common.maxIter;  po.gamma = 0.8;
        [path, info] = fmtstar_nd(env.bounds, env.start, env.goal, isFree, po);
    case 'BIT*'
        po = common;  po.maxNodes = common.maxIter;  po.gamma = 0.9;
        [path, info] = bitstar_nd(env.bounds, env.start, env.goal, isFree, po);
    case 'RRT#'
        [path, info] = rrtsharp_nd(env.bounds, env.start, env.goal, isFree, common);
    case 'APF-only'
        ao = common;  ao.useFallback = false;
        [path, info] = stable_apf2d(env, ao);
    case 'AB-IRRT*'
        bo = common;  bo.pbias = 0.5;  bo.dynamicRewire = true;  bo.rewireMax = 16;
        [~, ~, det] = bair_core2d(mode, bo);
        path = det.path;  info = det;
    otherwise
        error('smoke_test:unknown', 'unknown algorithm %s', name);
end
if isfield(info, 'success') && ~isempty(info.success)
    ok = logical(info.success);
end
end


% -------------------------------------------------------------------------
function free = report(name, path, ok, ms, env)
%REPORT  Print one line: success, length, runtime, and whether the path is collision free.
ok = logical(ok) && size(path, 1) > 1;
if ok
    L = path_length(path);
    free = all(collisionChecking(path(1:end - 1, :), path(2:end, :), env.squareAll, env.round));
else
    L = NaN;  free = false;
end
fprintf('  %-16s %6d %12.2f %10.1f %8d\n', name, ok, L, ms, free);
end


% -------------------------------------------------------------------------
function L = path_length(p)
%PATH_LENGTH  Sum of the Euclidean segments of a polyline.
d = diff(p);
L = sum(sqrt(sum(d .^ 2, 2)));
end
