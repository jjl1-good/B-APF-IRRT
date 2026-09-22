function regen_summaries()
%REGEN_SUMMARIES 由 results_*.csv 重新生成全部 summary_*.csv（完整统计口径）。
%
% 用途：当汇总脚本被同名旧文件遮蔽、或需要在不重跑实验的前提下刷新统计表时使用。
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));

jobs = {fullfile(entryDir, 'results_2d_v2.csv'), fullfile(entryDir, 'summary_2d.csv')};
modes = {'general', 'narrow', 'suspended', 'ring', 'overhang'};
for i = 1:numel(modes)
    jobs(end + 1, :) = {fullfile(entryDir, sprintf('results_3d_v2_%s.csv', modes{i})), ...
        fullfile(entryDir, sprintf('summary_3d_%s.csv', modes{i}))}; %#ok<AGROW>
end

for i = 1:size(jobs, 1)
    src = jobs{i, 1};
    if ~exist(src, 'file')
        fprintf('[skip] %s (missing)\n', src);
        continue
    end
    o = detectImportOptions(src);
    o.Encoding = 'UTF-8';
    o.VariableNamingRule = 'preserve';
    T = readtable(src, o);
    summarize_core(T, 'AB-IRRT*', jobs{i, 2});
    fprintf('[done] %s -> %s\n', src, jobs{i, 2});
end
fprintf('regen_summaries: finished.\n');
end
