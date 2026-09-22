function analyze_convergence()
%ANALYZE_CONVERGENCE 从 anytime 历史中提取“每个算法收敛到目标代价所需的时间与采样数”。
%
% 目标代价 target：该场景的“已知最优”= 八种算法 50 次运行平均长度中的最小值
%   （二维读 summary_2d.csv，三维读 summary_3d_<scene>.csv）。
% 对每次运行，求代价首次 ≤ target*(1+eps) 的时刻 t*（eps = 2 % / 5 %）：
%   reached 比例、达标运行的中位时间；
%   扩展次数按“预算 / 总时间”线性换算：n* ≈ t* × budget / timeTotal（近似）。
% 历史只有 1 个记录点的算法（如 PRM*/FMT* 只在末期记录）单独标注，其时间不可比。
%
% 输出：实验入口/convergence_2d.csv、convergence_3d.csv + 控制台表格。

entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir); addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));

algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
epss = [0.02, 0.05];

jobs = {  % dim, 场景名(与 anytime 中一致), anytime 文件, summary 文件
    '2d', '一般', 'anytime_2d_v2.mat',        'summary_2d.csv';
    '2d', '狭窄', 'anytime_2d_v2.mat',        'summary_2d.csv';
    '2d', '广阔', 'anytime_2d_v2.mat',        'summary_2d.csv';
    '3d', '一般', 'anytime_3d_v2_general.mat',   'summary_3d_general.csv';
    '3d', '狭窄', 'anytime_3d_v2_narrow.mat',    'summary_3d_narrow.csv';
    '3d', '悬空', 'anytime_3d_v2_suspended.mat', 'summary_3d_suspended.csv';
    '3d', '环廊', 'anytime_3d_v2_ring.mat',      'summary_3d_ring.csv';
    '3d', '悬挑', 'anytime_3d_v2_overhang.mat',  'summary_3d_overhang.csv'};

rows = struct('dim', {}, 'scenario', {}, 'algorithm', {}, 'eps', {}, 'target', {}, ...
    'runs', {}, 'reached', {}, 'medianTime', {}, 'estExtensions', {}, 'preExist', {}, 'note', {});
z = 0;
for j = 1:size(jobs, 1)
    dim = jobs{j, 1}; sc = jobs{j, 2};
    if strcmp(dim, '2d'), bud = 2000; else, bud = 6000; end
    mf = fullfile(entryDir, jobs{j, 3});
    sf = fullfile(entryDir, jobs{j, 4});
    if ~exist(mf, 'file') || ~exist(sf, 'file'), continue; end
    T = readtable(sf, 'Encoding', 'UTF-8');
    Tm = T(strcmp(T.scenario, sc), :);
    len = Tm.meanLength; len = len(isfinite(len) & len > 0);
    if isempty(len), continue; end
    target = min(len);
    S = load(mf); A = S.anytime;
    fprintf('\n===== %s %s (target = %.1f, budget = %d) =====\n', upper(dim), sc, target, bud);
    fprintf('%-15s %8s %10s %12s %12s %10s\n', 'algorithm', 'eps', 'reached', 'median t*[s]', 'est. iter N*', 'note');
    for a = 1:numel(algs)
        alg = algs{a};
        sel = strcmp({A.scenario}, sc) & strcmp({A.algorithm}, alg);
        runs = A(sel);
        if isempty(runs), continue; end
        tt = Tm(strcmp(Tm.algorithm, alg), :);
        rate = NaN; note = '';
        if ~isempty(tt) && isfinite(tt.meanTime) && tt.meanTime > 0
            rate = bud / tt.meanTime;      % 扩展次数/秒（场景级均值）
        end
        npt = nan(numel(runs), 1);
        for k = 1:numel(runs)
            npt(k) = numel(runs(k).t);
        end
        if all(npt <= 1)
            note = 'history records only the final solution (time not comparable)';
        end
        pre = mean(npt <= 1);
        % 仅对“预算 = 迭代次数”的算法，时间→扩展次数的线性换算才有意义
        iterOK = ismember(alg, {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'RRT#', 'BIT*'});
        if ~iterOK && isempty(note)
            note = 'sample-to-time mapping not applicable';
        end
        for e = epss
            tStar = nan(numel(runs), 1);
            for k = 1:numel(runs)
                t = runs(k).t(:); c = runs(k).cost(:);
                ok = isfinite(t) & isfinite(c);
                t = t(ok); c = c(ok);
                if isempty(t), continue; end
                c = cummin(c);
                idx = find(c <= target * (1 + e), 1, 'first');
                if ~isempty(idx), tStar(k) = t(idx); end
            end
            good = tStar(isfinite(tStar));
            z = z + 1;
            rows(z) = struct('dim', dim, 'scenario', sc, 'algorithm', alg, 'eps', e, ...
                'target', target, 'runs', numel(runs), 'reached', mean(isfinite(tStar)), ...
                'medianTime', median(good), 'estExtensions', median(good) * rate * iterOK, ...
                'preExist', pre, 'note', note);
            if iterOK
                nStr = sprintf('%12.0f', median(good) * rate);
            else
                nStr = sprintf('%12s', '-');
            end
            fprintf('%-15s %7.0f%% %9.0f%% %12.4f %s  %s\n', alg, 100 * e, ...
                100 * mean(isfinite(tStar)), median(good), nStr, note);
        end
    end
end

C = struct2table(rows);
writetable(C, fullfile(entryDir, 'convergence_2d.csv'), 'Encoding', 'UTF-8');
% 三维单独存一份（便于附录引用）
C3 = C(strcmp(C.dim, '3d'), :);
writetable(C3, fullfile(entryDir, 'convergence_3d.csv'), 'Encoding', 'UTF-8');
fprintf('\nwrote convergence_2d.csv (%d rows) and convergence_3d.csv (%d rows)\n', height(C), height(C3));
end
