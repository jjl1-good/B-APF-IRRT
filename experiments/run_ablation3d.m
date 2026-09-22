function results = run_ablation3d(runs, opts)
%RUN_ABLATION3D AB-IRRT* 机制消融实验（三维，3 变体 × 2 场景 × runs 次）。
% 变体：APF-only、Informed-RRT*、AB-IRRT* (full)。
% 场景：general（原一般）与 ring（环形柱廊，真三维）。
% 输出：ablation_3d.csv（UTF-8）。

if nargin < 1 || isempty(runs), runs = 50; end
if nargin < 2, opts = struct; end
def = struct('maxIter', 6000, 'step', 6, 'goalBias', 0.08, 'rewireRadius', 18, ...
    'rewireMax', 30, 'pbias', 0.1, 'outputFile', '', 'quiet', false);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));
if isempty(opts.outputFile), opts.outputFile = fullfile(entryDir, 'ablation_3d.csv'); end

variants = {'APF-only', 'Informed-RRT*', 'w/o seed shortcutting', 'w/o ellipse seeding', ...
    'w/o path-bias sampling', 'w/o dynamic rewiring', 'AB-IRRT* (full)'};
modes = {'general_v2', 'ring'};
names = {'一般', '环廊'};

rows = struct('variant', {}, 'scenario', {}, 'run', {}, 'seed', {}, 'success', {}, ...
    'length', {}, 'time', {}, 'firstSolutionTime', {}, 'tPre', {}, 'tOpt', {}, ...
    'nodes', {}, 'turnSum', {}, 'minClear', {}, 'escapes', {}, 'sideFlips', {}, 'fallback', {});
z = 0; baseSeed = 20260903;
for c = 1:numel(modes)
    env = bair_env3d(modes{c});
    qf = @(pts) query_pts3(pts, env);
    for r = 1:runs
        seed = baseSeed + 1000 * c + r;
        for v = 1:numel(variants)
            rng(seed, 'twister');
            res = run_variant(variants{v}, env, modes{c}, opts);
            z = z + 1;
            m = metrics_of(res.path, qf);
            rows(z) = struct('variant', variants{v}, 'scenario', names{c}, 'run', r, ...
                'seed', seed, 'success', res.success, 'length', res.length, 'time', res.time, ...
                'firstSolutionTime', res.firstSol, 'tPre', res.tPre, 'tOpt', res.tOpt, ...
                'nodes', res.nodes, 'turnSum', m.turnSum, 'minClear', m.minClear, ...
                'escapes', res.escapes, 'sideFlips', res.flips, 'fallback', res.fb);
        end
        if ~opts.quiet && mod(r, max(1, floor(runs / 5))) == 0
            log_progress('  3D ablation %s: run %d/%d done', names{c}, r, runs);
        end
    end
    if ~opts.quiet, log_progress('3D ablation scene %s complete.', names{c}); end
end
results = struct2table(rows);
writetable(results, opts.outputFile, 'Encoding', 'UTF-8');
fprintf('3D ablation complete -> %s\n', opts.outputFile);
end

function res = run_variant(variant, env, mode, opts)
res = struct('success', false, 'length', inf, 'time', NaN, 'firstSol', NaN, ...
    'tPre', NaN, 'tOpt', NaN, 'nodes', NaN, 'escapes', NaN, 'flips', NaN, 'fb', false, ...
    'path', zeros(0, 3));

% 与主管线一致的开关：默认全开，逐个变体只关一项
sw = struct('pbias', opts.pbias, 'dynamicRewire', true, 'useSeedEllipse', true, 'seedShortcut', true);
for f = fieldnames(sw)'
    if isfield(opts, f{1}), sw.(f{1}) = opts.(f{1}); end
end
switch variant
    case {'w/o ellipse seeding'}
        sw.useSeedEllipse = false;
    case {'w/o path-bias sampling'}
        sw.pbias = 0;
    case {'w/o dynamic rewiring'}
        sw.dynamicRewire = false;
    case {'w/o seed shortcutting'}
        sw.seedShortcut = false;
end

switch variant
    case 'APF-only'
        ao = struct('useFallback', false, 'step', opts.step);
        t0 = tic; [p, ai] = stable_apf3d(env, ao); tt = toc(t0);
        res.success = ai.success; res.path = p; res.length = ai.length;
        res.time = tt; res.tPre = tt; res.tOpt = 0;
        if ai.success, res.firstSol = ai.timeLocal; end
        res.escapes = ai.escapes; res.flips = ai.sideFlips; res.fb = ai.fallback;
    case 'Informed-RRT*'
        ro = struct('maxIter', opts.maxIter, 'step', opts.step, 'goalBias', opts.goalBias, ...
            'rewireRadius', opts.rewireRadius);
        t0 = tic; [p, ri] = informed_rrtstar3d(env, [], ro); tt = toc(t0);
        res.tPre = 0; res.tOpt = tt; res.time = tt;
        res.path = p; res.success = logical(ri.success) && size(p, 1) >= 2;
        res.nodes = ri.nodes; res.firstSol = ri.firstSolutionTime;
        if ~res.success, res.length = inf; else, res.length = path_length3(p); end
    case {'w/o seed shortcutting', 'w/o ellipse seeding', 'w/o path-bias sampling', 'w/o dynamic rewiring', 'AB-IRRT* (full)'}
        bo = struct('maxIter', opts.maxIter, 'step', opts.step, 'goalBias', opts.goalBias, ...
            'rewireRadius', opts.rewireRadius, 'pbias', sw.pbias, ...
            'dynamicRewire', sw.dynamicRewire, 'rewireMax', opts.rewireMax, ...
            'useSeedEllipse', sw.useSeedEllipse, 'seedShortcut', sw.seedShortcut, ...
            'seedShortcutRatio', 1.15, 'useFallback', true);
        t0 = tic; [~, ~, d] = bair_core3d(mode, bo); tt = toc(t0);
        res.path = d.path; res.success = d.success; res.length = d.length;
        res.time = tt; res.tPre = d.tPre; res.tOpt = d.tOpt;
        res.firstSol = d.firstSolutionTime; res.nodes = d.rrt.nodes;
        res.escapes = d.apf.escapes; res.flips = d.apf.sideFlips; res.fb = d.apf.fallback;
    otherwise
        error('unknown ablation variant: %s', variant);
end
end

function m = metrics_of(path, qf)
m = struct('turnSum', NaN, 'minClear', NaN);
if size(path, 1) >= 2
    pm = path_metrics_nd(path, qf);
    m.turnSum = pm.turnSum; m.minClear = pm.minClear;
end
end

function d = query_pts3(pts, env)
d = zeros(size(pts, 1), 1);
for i = 1:size(pts, 1), d(i) = obstacle_query3d(pts(i, :), env); end
end

function L = path_length3(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
