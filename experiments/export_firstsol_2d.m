function export_firstsol_2d()
%EXPORT_FIRSTSOL_2D 汇总四个算法的"首解"信息（时间与长度）到 firstsol_2d.csv。
%   数据来源：ge_anytime_2d.mat（葛超2025 复现）、anytime_2d_v2.mat（主实验，含 AB-IRRT*、
%   Informed-RRT*）、hybrid_anytime_2d_main.mat（HAS-RRT 等）。
%   anytime 历史的第一项即首解：(t, cost)。
entryDir = fileparts(mfilename('fullpath'));
files = {'ge_anytime_2d.mat', 'ge_anytime_2d_uniform.mat', 'anytime_2d_v2.mat', ...
         'hybrid_anytime_2d_main.mat', 'has_star_anytime_2d.mat'};
rows = {};
for f = 1:numel(files)
    p = fullfile(entryDir, files{f});
    if ~exist(p, 'file'), fprintf('skip %s\n', files{f}); continue; end
    S = load(p, 'anytime');
    A = S.anytime;
    for i = 1:numel(A)
        a = A(i);
        if isempty(a.t) || isempty(a.cost), continue; end
        rows(end + 1, :) = {a.scenario, a.algorithm, a.run, a.t(1), a.cost(1), size(a.t, 1)}; %#ok<AGROW>
    end
end
T = cell2table(rows, 'VariableNames', {'scenario', 'algorithm', 'run', ...
    'firstSolTime', 'firstSolLength', 'nHist'});
writetable(T, fullfile(entryDir, 'firstsol_2d.csv'), 'Encoding', 'UTF-8');
fprintf('firstsol_2d.csv: %d rows\n', height(T));
end
