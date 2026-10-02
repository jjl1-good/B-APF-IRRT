function run_has_star_2d(runs)
%RUN_HAS_STAR_2D 跑"HAS-RRT*"与"HAS-RRT*-Informed"（骨架引导 + RRT* 机制）的二维对照。
%   与主实验同场景、同种子、同预算；输出 has_star_2d.csv。
if nargin < 1 || isempty(runs), runs = 50; end
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir); addpath(fullfile(rootDir, 'common')); addpath(fullfile(rootDir, 'common3d'));
o = struct('algs', {{'HAS-RRT*', 'HAS-RRT*-Informed'}}, 'quiet', true, ...
    'outputFile', fullfile(entryDir, 'has_star_2d.csv'), ...
    'anytimeFile', fullfile(entryDir, 'has_star_anytime_2d.mat'));
run_2d_comparison(runs, o);
fprintf('HAS-RRT* 2-D comparison done (%d runs)\n', runs);
end
