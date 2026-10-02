function warm_run2d()
%WARM_RUN2D 二维热启动等价基线的正式实验。
%   (a) 主组：50 次配对，含无热启动参照 Informed-RRT*
%   (b) 批处理组：10 次（BIT* 单次数秒至数十秒，单独控重复数）
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir);
addpath(fullfile(rootDir, 'common'));
addpath(fullfile(rootDir, 'common3d'));

A = {'Informed-RRT*', 'Warm-Informed-RRT*', 'AB-IRRT*'};
o = struct('algs', {A}, 'quiet', true, ...
    'outputFile', fullfile(entryDir, 'warmstart_2d_main.csv'), ...
    'anytimeFile', fullfile(entryDir, 'warmstart_anytime_2d_main.mat'));
run_2d_comparison(50, o);

B = {'BIT*', 'Warm-BIT*'};
o2 = struct('algs', {B}, 'quiet', true, ...
    'outputFile', fullfile(entryDir, 'warmstart_2d_bit.csv'), ...
    'anytimeFile', fullfile(entryDir, 'warmstart_anytime_2d_bit.mat'));
run_2d_comparison(10, o2);

fprintf('warm_run2d done\n');
end
