function hybrid_run2d(runs)
%HYBRID_RUN2D 二维文献混合基线组实验（同一套种子与预算）。
%   Informed-RRT*（无引导参照）/ APF-IRRT*（Wu et al. 2022）/
%   HAS-RRT（Uwacu et al. 2025）/ AB-IRRT*（本文方法）
if nargin < 1 || isempty(runs), runs = 50; end
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir);
addpath(fullfile(rootDir, 'common'));
addpath(fullfile(rootDir, 'common3d'));

A = {'Informed-RRT*', 'APF-IRRT*', 'HAS-RRT', 'AB-IRRT*'};
o = struct('algs', {A}, 'quiet', true, ...
    'outputFile', fullfile(entryDir, 'hybrid_2d_main.csv'), ...
    'anytimeFile', fullfile(entryDir, 'hybrid_anytime_2d_main.mat'));
run_2d_comparison(runs, o);
fprintf('hybrid_run2d done\n');
end
