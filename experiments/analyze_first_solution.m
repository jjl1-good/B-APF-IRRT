function analyze_first_solution()
%ANALYZE_FIRST_SOLUTION 从 anytime 历史中提取“首解路径长度”并做统计对比。
%   首解长度 = 该次运行最早被记录到的可行解代价（cost 的第一个元素）。
%   对 AB-IRRT* 而言就是局部预规划路径的代价；对采样类基线则是其第一次找到可行解的长度。
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir); addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));

algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
jobs = {fullfile(entryDir, 'anytime_2d_v2.mat'), '2d';
        fullfile(entryDir, 'anytime_3d_v2_general.mat'), '3d';
        fullfile(entryDir, 'anytime_3d_v2_narrow.mat'), '3d';
        fullfile(entryDir, 'anytime_3d_v2_suspended.mat'), '3d';
        fullfile(entryDir, 'anytime_3d_v2_ring.mat'), '3d';
        fullfile(entryDir, 'anytime_3d_v2_overhang.mat'), '3d'};

rows = struct('dim', {}, 'scenario', {}, 'algorithm', {}, 'run', {}, 'firstLen', {});
for j = 1:size(jobs, 1)
    f = jobs{j, 1};
    if ~exist(f, 'file'), continue; end
    S = load(f);
    a = S.anytime;
    for i = 1:numel(a)
        c = a(i).cost;
        if isempty(c), continue; end
        c = c(isfinite(c));
        if isempty(c), continue; end
        rows(end + 1) = struct('dim', jobs{j, 2}, 'scenario', a(i).scenario, ...
            'algorithm', a(i).algorithm, 'run', a(i).run, 'firstLen', c(1)); %#ok<AGROW>
    end
end
T = struct2table(rows);
writetable(T, fullfile(entryDir, 'first_solution_length.csv'), 'Encoding', 'UTF-8');

scenes2 = {'一般', '狭窄', '广阔'};
scenes3 = {'一般', '狭窄', '悬空', '环廊', '悬挑'};
fprintf('\n%-6s %-8s' , 'dim', 'scene');
fprintf('%16s', algs{:});
fprintf('\n');
for dim = {'2d', '3d'}
    list = scenes2; if strcmp(dim{1}, '3d'), list = scenes3; end
    for s = 1:numel(list)
        sc = list{s};
        fprintf('%-6s %-8s', dim{1}, sc);
        for k = 1:numel(algs)
            m = strcmp(T.dim, dim{1}) & strcmp(T.scenario, sc) & strcmp(T.algorithm, algs{k});
            v = T.firstLen(m);
            if isempty(v)
                fprintf('%16s', '—');
            else
                fprintf('%16s', sprintf('%.1f±%.1f', mean(v), std(v)));
            end
        end
        fprintf('\n');
    end
end

% ---- 与 AB-IRRT* 的配对比较（仅双方都成功的运行） ----
fprintf('\n--- paired comparison of first-solution length vs AB-IRRT* ---\n');
fprintf('%-6s %-8s %-16s %10s %10s %10s %10s\n', 'dim', 'scene', 'algorithm', 'AB_mean', 'alg_mean', 'delta', 'p');
summary = {};
for dim = {'2d', '3d'}
    list = scenes2; if strcmp(dim{1}, '3d'), list = scenes3; end
    for s = 1:numel(list)
        sc = list{s};
        ma = strcmp(T.dim, dim{1}) & strcmp(T.scenario, sc) & strcmp(T.algorithm, 'AB-IRRT*');
        if ~any(ma), continue; end
        A = T(ma, :);
        for k = 2:numel(algs)
            mb = strcmp(T.dim, dim{1}) & strcmp(T.scenario, sc) & strcmp(T.algorithm, algs{k});
            B = T(mb, :);
            if isempty(B), continue; end
            [~, ia, ib] = intersect(A.run, B.run);
            if numel(ia) < 5, continue; end
            x = A.firstLen(ia); y = B.firstLen(ib);
            [d, ~] = cliffs_delta(x, y);
            p = permutation_test(x, y);
            summary(end + 1, :) = {dim{1}, sc, algs{k}, mean(x), mean(y), d, p}; %#ok<AGROW>
            fprintf('%-6s %-8s %-16s %10.1f %10.1f %10.2f %10.4f\n', ...
                dim{1}, sc, algs{k}, mean(x), mean(y), d, p);
        end
    end
end
S = cell2table(summary, 'VariableNames', {'dim', 'scenario', 'algorithm', 'AB_mean', 'alg_mean', 'cliffs_delta', 'p'});
writetable(S, fullfile(entryDir, 'first_solution_vs_AB.csv'), 'Encoding', 'UTF-8');
log_progress('=== analyze_first_solution DONE ===');
end
