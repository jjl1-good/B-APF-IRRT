function run_ge_2d(runs)
%RUN_GE_2D 只跑"葛超等 2025（改进 Informed-RRT*）"的等价复现，与主实验同场景、同种子、同预算。
%   输出：ge_2025_2d.csv（主口径：APF 概率选点）与 ge_2025_2d_uniform.csv（口径敏感性对照：
%   仅保留式(12)的启发式父节点选择、采样退回全域均匀）。
%   用法：run_ge_2d(50)
if nargin < 1 || isempty(runs), runs = 50; end
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir); addpath(fullfile(rootDir, 'common')); addpath(fullfile(rootDir, 'common3d'));

o = struct('algs', {{'Ge-IRRT*'}}, 'quiet', true, ...
    'outputFile', fullfile(entryDir, 'ge_2025_2d.csv'), ...
    'anytimeFile', fullfile(entryDir, 'ge_anytime_2d.mat'));
run_2d_comparison(runs, o);

o2 = struct('algs', {{'Ge-IRRT*-uniform'}}, 'quiet', true, ...
    'outputFile', fullfile(entryDir, 'ge_2025_2d_uniform.csv'), ...
    'anytimeFile', fullfile(entryDir, 'ge_anytime_2d_uniform.mat'));
run_2d_comparison(runs, o2);

fprintf('ge 2-D comparison done (%d runs)\n', runs);
end
