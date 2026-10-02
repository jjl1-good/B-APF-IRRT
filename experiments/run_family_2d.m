function run_family_2d(runs)
%RUN_FAMILY_2D 同一批次跑"同族 warm-start 方法"的二维对照（论文新增 3.5 节 + 附表 S20 用）。
%   同族 = 先构造首解/引导结构再优化：葛超2025（APF 选点 + 双向贪心 + 动态步长 + B 样条）、
%   HAS-RRT（原文骨架可行版）、HAS-RRT*（本文构造：骨架 + RRT* 机制）、
%   HAS-RRT*-Informed（再加椭圆采样）；参照本文 AB-IRRT*。同场景、同种子、同预算，同一进程内运行，
%   因此运行时间可比。
if nargin < 1 || isempty(runs), runs = 50; end
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir); addpath(fullfile(rootDir, 'common')); addpath(fullfile(rootDir, 'common3d'));
o = struct('algs', {{'AB-IRRT*', 'Ge-IRRT*', 'HAS-RRT', 'HAS-RRT*', 'HAS-RRT*-Informed'}}, ...
    'quiet', true, 'outputFile', fullfile(entryDir, 'family_2d.csv'), ...
    'anytimeFile', fullfile(entryDir, 'family_anytime_2d.mat'));
run_2d_comparison(runs, o);
fprintf('family 2-D comparison done (%d runs)\n', runs);
end
