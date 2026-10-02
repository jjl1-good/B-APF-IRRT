function R = run_warmstart_comparison(dim, runs, opts)
%RUN_WARMSTART_COMPARISON 热启动“数学等价基线”对比实验。
%
%   审稿意见：AB-IRRT* 为何不与其它 hybrid / warm-started 规划器做
%   数学上等价的基线对比？
%
%   本实验给出两条等价基线，它们与 AB-IRRT* 的差别恰好就是本文的增强项：
%     Warm-Informed-RRT* : 同一条 Bug-APF 预规划路径仅用作初始代价上界
%                          （立即打开采样椭圆），不加路径偏置采样、
%                          不用自由空间采样、不改重布线半径；
%     Warm-BIT*          : 同一条预规划路径仅用于初始化 BIT* 的 c_best，
%                          其余完全按原版 BIT* 运行。
%   两条基线都与 AB-IRRT* 共用同一次预规划（同参数、同随机种子），
%   因此属于“同起点、只差增强项”的等价比较。
%
%   用法：
%     R = run_warmstart_comparison(2, 50);        % 二维，3 场景 × 50 次
%     R = run_warmstart_comparison(3, 50);        % 三维，5 场景 × 50 次
%     R = run_warmstart_comparison(2, 50, struct('algs', {{'Warm-BIT*', 'AB-IRRT*'}}));
%
%   输出：warmstart_2d.csv / warmstart_3d.csv 与对应的 anytime .mat

if nargin < 1 || isempty(dim), dim = 2; end
if nargin < 2 || isempty(runs), runs = 50; end
if nargin < 3, opts = struct; end

entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir);
addpath(fullfile(rootDir, 'common'));
addpath(fullfile(rootDir, 'common3d'));

algs = {'Informed-RRT*', 'BIT*', 'Warm-Informed-RRT*', 'Warm-BIT*', 'AB-IRRT*'};
if isfield(opts, 'algs') && ~isempty(opts.algs), algs = opts.algs; end

o = struct('algs', {algs}, 'quiet', true);
if isfield(opts, 'quiet'), o.quiet = opts.quiet; end

if dim == 2
    o.outputFile = fullfile(entryDir, 'warmstart_2d.csv');
    o.anytimeFile = fullfile(entryDir, 'warmstart_anytime_2d.mat');
    R = run_2d_comparison(runs, o);
else
    o.outputFile = fullfile(entryDir, 'warmstart_3d.csv');
    o.anytimeFile = fullfile(entryDir, 'warmstart_anytime_3d.mat');
    if isfield(opts, 'sceneFilter') && ~isempty(opts.sceneFilter)
        o.scenes = opts.sceneFilter;
    end
    R = run_3d_comparison(runs, o);
end
fprintf('warm-start comparison (%d-D, %d runs) -> %s\n', dim, runs, o.outputFile);
end
