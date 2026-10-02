function warm_run3d()
%WARM_RUN3D 三维热启动等价基线的正式实验。
%   (a) 主组：50 次配对，含无热启动参照 Informed-RRT*
%   (b) 批处理组：5 次（BIT* 单次 11-117 s，重复数相应下调）
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir);
addpath(fullfile(rootDir, 'common'));
addpath(fullfile(rootDir, 'common3d'));

A = {'Informed-RRT*', 'Warm-Informed-RRT*', 'AB-IRRT*'};
o = struct('algs', {A}, 'quiet', true, ...
    'outputFile', fullfile(entryDir, 'warmstart_3d_main.csv'), ...
    'anytimeFile', fullfile(entryDir, 'warmstart_anytime_3d_main.mat'));
run_3d_comparison(50, o);

B = {'BIT*', 'Warm-BIT*'};
o2 = struct('algs', {B}, 'quiet', true, ...
    'outputFile', fullfile(entryDir, 'warmstart_3d_bit.csv'), ...
    'anytimeFile', fullfile(entryDir, 'warmstart_anytime_3d_bit.mat'));
run_3d_comparison(3, o2);

fprintf('warm_run3d done\n');
end
