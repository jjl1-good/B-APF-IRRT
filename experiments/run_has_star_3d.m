function run_has_star_3d(runs)
%RUN_HAS_STAR_3D 跑"HAS-RRT*"与"HAS-RRT*-Informed"的三维对照（逐场景单独调用，c=1，种子与主实验一致）。
%   输出 has_star_3d_<tag>.csv（正式表由 _merge_has_star_3d.py 合并为 has_star_3d.csv）。
if nargin < 1 || isempty(runs), runs = 50; end
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir); addpath(fullfile(rootDir, 'common')); addpath(fullfile(rootDir, 'common3d'));
scenes = {'general_v2', 'narrow', 'suspended', 'ring', 'overhang'};
tags = {'general', 'narrow', 'suspended', 'ring', 'overhang'};
algs = {'HAS-RRT*', 'HAS-RRT*-Informed'};
for i = 1:numel(scenes)
    o = struct('algs', {algs}, 'quiet', true, 'scenes', {{scenes{i}}}, ...
        'outputFile', fullfile(entryDir, sprintf('has_star_3d_%s.csv', tags{i})), ...
        'anytimeFile', fullfile(entryDir, sprintf('has_star_anytime_3d_%s.mat', tags{i})));
    fprintf('--- HAS-RRT* / %s ---\n', tags{i});
    run_3d_comparison(runs, o);
end
fprintf('HAS-RRT* 3-D comparison done (%d runs)\n', runs);
end
