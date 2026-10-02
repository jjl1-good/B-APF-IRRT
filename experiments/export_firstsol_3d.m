function export_firstsol_3d()
%EXPORT_FIRSTSOL_3D 汇总三维对比中各算法的"首解"信息（时间与长度）到 firstsol_3d.csv。
%   来源：ge_2025_3d_anytime_<tag>.mat / ge_2025_3d_uniform_anytime_<tag>.mat（葛超2025 复现）、
%         anytime_3d_v2_<tag>.mat（主实验）、_rerun\hybrid_anytime_3d_<tag>.mat（混合基线组）。
%   anytime 历史第一项即首解 (t, cost)。
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
rerunDir = fullfile(rootDir, '..', '修改稿', '_rerun');
tags = {'general', 'narrow', 'suspended', 'ring', 'overhang'};
rows = {};
srcs = {};
for i = 1:numel(tags)
    srcs{end + 1} = fullfile(entryDir, sprintf('ge_2025_3d_anytime_%s.mat', tags{i}));      %#ok<AGROW>
    srcs{end + 1} = fullfile(entryDir, sprintf('ge_2025_3d_uniform_anytime_%s.mat', tags{i})); %#ok<AGROW>
    srcs{end + 1} = fullfile(entryDir, sprintf('anytime_3d_v2_%s.mat', tags{i}));            %#ok<AGROW>
    srcs{end + 1} = fullfile(rerunDir, sprintf('hybrid_anytime_3d_%s.mat', tags{i}));        %#ok<AGROW>
    srcs{end + 1} = fullfile(entryDir, sprintf('has_star_anytime_3d_%s.mat', tags{i}));      %#ok<AGROW>
end
seen = {};
for f = 1:numel(srcs)
    p = srcs{f};
    if ~exist(p, 'file'), fprintf('skip %s\n', p); continue; end
    if any(strcmp(seen, p)), continue; end
    seen{end + 1} = p; %#ok<AGROW>
    try
        S = load(p, 'anytime');
    catch
        fprintf('load failed: %s\n', p); continue;
    end
    A = S.anytime;
    for i = 1:numel(A)
        a = A(i);
        if isempty(a.t) || isempty(a.cost), continue; end
        rows(end + 1, :) = {a.scenario, a.algorithm, a.run, a.t(1), a.cost(1), size(a.t, 1)}; %#ok<AGROW>
    end
end
T = cell2table(rows, 'VariableNames', {'scenario', 'algorithm', 'run', ...
    'firstSolTime', 'firstSolLength', 'nHist'});
writetable(T, fullfile(entryDir, 'firstsol_3d.csv'), 'Encoding', 'UTF-8');
fprintf('firstsol_3d.csv: %d rows\n', height(T));
end
