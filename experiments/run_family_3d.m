function run_family_3d(runs)
%RUN_FAMILY_3D 同一批次跑"同族 warm-start 方法"的三维对照（论文附表 S21 用）。
%   同族 = 葛超2025 的三维推广、HAS-RRT（原文）、HAS-RRT*（本文构造）、HAS-RRT*-Informed；
%   参照本文 AB-IRRT*。逐场景单独调用（c=1），种子与主实验一致。
if nargin < 1 || isempty(runs), runs = 50; end
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir); addpath(fullfile(rootDir, 'common')); addpath(fullfile(rootDir, 'common3d'));
scenes = {'general_v2', 'narrow', 'suspended', 'ring', 'overhang'};
tags = {'general', 'narrow', 'suspended', 'ring', 'overhang'};
algs = {'AB-IRRT*', 'Ge-IRRT*', 'HAS-RRT', 'HAS-RRT*', 'HAS-RRT*-Informed'};
for i = 1:numel(scenes)
    o = struct('algs', {algs}, 'quiet', true, 'scenes', {{scenes{i}}}, ...
        'outputFile', fullfile(entryDir, sprintf('family_3d_%s.csv', tags{i})), ...
        'anytimeFile', fullfile(entryDir, sprintf('family_anytime_3d_%s.mat', tags{i})));
    fprintf('--- family 3-D / %s ---\n', tags{i});
    run_3d_comparison(runs, o);
end
fprintf('family 3-D comparison done (%d runs)\n', runs);
end
