function hybrid_run3d(runs)
%HYBRID_RUN3D 三维文献混合基线组实验（同一套种子与预算）。
if nargin < 1 || isempty(runs), runs = 50; end
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir);
addpath(fullfile(rootDir, 'common'));
addpath(fullfile(rootDir, 'common3d'));

A = {'Informed-RRT*', 'APF-IRRT*', 'HAS-RRT', 'AB-IRRT*'};
o = struct('algs', {A}, 'quiet', true, ...
    'outputFile', fullfile(entryDir, 'hybrid_3d_main.csv'), ...
    'anytimeFile', fullfile(entryDir, 'hybrid_anytime_3d_main.mat'));
run_3d_comparison(runs, o);
fprintf('hybrid_run3d done\n');
end
