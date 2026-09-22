function make_extra_figures()
%MAKE_EXTRA_FIGURES 附加分析图：首解路径长度 + 二维预算敏感性。
%   输出到 figures：
%     fig_firstlen_2d.png / fig_firstlen_3d.png   首解路径长度（9 算法）
%     fig_budget_2d.png                           二维预算敏感性（500/1000/2000）
%   并写出汇总 CSV：first_solution_length_summary.csv / budget_sweep_summary.csv
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
wsRoot = fileparts(rootDir);
outDir = fullfile(rootDir, 'figures');
if ~exist(outDir, 'dir'), mkdir(outDir); end
addpath(entryDir); addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));

algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};

%% ---------- 1) 首解路径长度 ----------
T = readtable(fullfile(entryDir, 'first_solution_length.csv'), 'Encoding', 'UTF-8', 'VariableNamingRule', 'preserve');
groups = {{'2d', {'一般', '狭窄', '广阔'}, {'2-D general', '2-D narrow', '2-D expansive'}, 'fig_firstlen_2d'}, ...
          {'3d', {'一般', '狭窄', '悬空', '环廊', '悬挑'}, ...
           {'3-D general', '3-D narrow', '3-D suspended', '3-D ring', '3-D overhang'}, 'fig_firstlen_3d'}};
sumRows = {};
for g = 1:numel(groups)
    dim = groups{g}{1}; scenes = groups{g}{2}; names = groups{g}{3}; tag = groups{g}{4};
    nS = numel(scenes);
    fig = figure('Visible', 'off', 'Position', [40 40 max(700, 520 * nS) 520]);
    M = nan(nS, numel(algs));
    for s = 1:nS
        ax = subplot(1, nS, s); hold(ax, 'on'); box(ax, 'on');
        for k = 1:numel(algs)
            m = strcmp(T.dim, dim) & strcmp(T.scenario, scenes{s}) & strcmp(T.algorithm, algs{k});
            v = T.firstLen(m);
            if ~isempty(v), M(s, k) = mean(v); end
            sumRows(end + 1, :) = {dim, scenes{s}, algs{k}, numel(v), ...
                mean_or_nan(v), std_or_nan(v), median_or_nan(v)}; %#ok<AGROW>
        end
        b = bar(ax, 1:numel(algs), M(s, :), 0.72, 'FaceColor', 'flat');
        for k = 1:numel(algs), b.CData(k, :) = alg_color(algs{k}); end
        ymax = max(M(s, :));
        for k = 1:numel(algs)
            if isfinite(M(s, k))
                text(ax, k, M(s, k) + 0.02 * ymax, sprintf('%.0f', M(s, k)), ...
                    'HorizontalAlignment', 'center', 'FontSize', 8.5);
            end
        end
        ylim(ax, [0, ymax * 1.16]);
        set(ax, 'XTick', 1:numel(algs), 'XTickLabel', algs, 'XTickLabelRotation', 40, 'FontSize', 8.5);
        ylabel(ax, 'First-solution path length');
        title(ax, names{s}, 'FontSize', 12, 'FontWeight', 'bold');
        grid(ax, 'on');
    end
    sgtitle(fig, 'Path length of the first feasible solution (mean over 50 paired runs)', ...
        'FontSize', 13.5, 'FontWeight', 'bold');
    save_fig(fig, outDir, tag);
    close(fig);
end
S = cell2table(sumRows, 'VariableNames', {'dim', 'scenario', 'algorithm', 'runs', 'meanFirstLen', 'stdFirstLen', 'medianFirstLen'});
writetable(S, fullfile(entryDir, 'first_solution_length_summary.csv'), 'Encoding', 'UTF-8');

%% ---------- 2) 二维预算敏感性 ----------
budgets = [500, 1000, 2000];
files = {fullfile(entryDir, 'results_2d_b500.csv'), fullfile(entryDir, 'results_2d_b1000.csv'), ...
         fullfile(entryDir, 'results_2d_v2.csv')};
have = cellfun(@(f) exist(f, 'file') > 0, files);
if any(have)
    budgets = budgets(have); files = files(have);
    scenes = {'一般', '狭窄', '广阔'};
    names = {'2-D general', '2-D narrow', '2-D expansive'};
    L = nan(numel(budgets), numel(scenes), numel(algs));
    Suc = nan(numel(budgets), numel(scenes), numel(algs));
    Lp = nan(numel(budgets), numel(scenes), numel(algs));   % 三档都成功的配对子集均值
    Np = zeros(numel(budgets), numel(scenes), numel(algs)); % 配对子集大小
    keepRun = cell(numel(budgets), numel(scenes), numel(algs));
    bRows = {};
    for bi = 1:numel(budgets)
        Tb = readtable(files{bi}, 'Encoding', 'UTF-8', 'VariableNamingRule', 'preserve');
        for s = 1:numel(scenes)
            for k = 1:numel(algs)
                m = strcmp(Tb.scenario, scenes{s}) & strcmp(Tb.algorithm, algs{k});
                if ~any(m), continue; end
                ok = logical(Tb.success(m));
                v = double(Tb.length(m)); v = v(ok & isfinite(v));
                if ~isempty(v), L(bi, s, k) = mean(v); end
                Suc(bi, s, k) = 100 * mean(ok);
                bRows(end + 1, :) = {budgets(bi), scenes{s}, algs{k}, Suc(bi, s, k), ...
                    mean_or_nan(v), std_or_nan(v)}; %#ok<AGROW>
                % 记录“本档成功的 run -> 长度”，供配对子集使用
                rid = double(Tb.run(m)); vlen = double(Tb.length(m));
                d = containers.Map('KeyType', 'double', 'ValueType', 'double');
                for q = 1:numel(rid)
                    if ok(q) && isfinite(vlen(q)), d(rid(q)) = vlen(q); end
                end
                keepRun{bi, s, k} = d;
            end
        end
    end
    % 配对子集：三档都成功的同一批种子上的均值（预算即“更多时间”时这才是可比口径）
    for s = 1:numel(scenes)
        for k = 1:numel(algs)
            common = [];
            for bi = 1:numel(budgets)
                d = keepRun{bi, s, k};
                ks = []; if ~isempty(d), ks = cell2mat(keys(d)); end
                if bi == 1
                    common = ks;
                else
                    common = intersect(common, ks);
                end
            end
            for bi = 1:numel(budgets)
                d = keepRun{bi, s, k};
                if isempty(d) || isempty(common), continue; end
                vv = arrayfun(@(q) d(q), common);
                Lp(bi, s, k) = mean(vv); Np(bi, s, k) = numel(vv);
            end
        end
    end
    pRows = {};
    for s = 1:numel(scenes)
        for k = 1:numel(algs)
            for bi = 1:numel(budgets)
                if Np(bi, s, k) > 0
                    pRows(end + 1, :) = {budgets(bi), scenes{s}, algs{k}, ...
                        Np(bi, s, k), Lp(bi, s, k)}; %#ok<AGROW>
                end
            end
        end
    end
    P = cell2table(pRows, 'VariableNames', ...
        {'budget', 'scenario', 'algorithm', 'nPaired', 'meanLengthPaired'});
    writetable(P, fullfile(entryDir, 'budget_paired_summary.csv'), 'Encoding', 'UTF-8');
    B = cell2table(bRows, 'VariableNames', {'budget', 'scenario', 'algorithm', 'successRate', 'meanLength', 'stdLength'});
    writetable(B, fullfile(entryDir, 'budget_sweep_summary.csv'), 'Encoding', 'UTF-8');

    fig = figure('Visible', 'off', 'Position', [40 40 1560 520]);
    for s = 1:numel(scenes)
        ax = axes('Parent', fig, 'Position', [0.048 + (s - 1) * 0.222, 0.155, 0.192, 0.70]);
        hold(ax, 'on'); box(ax, 'on');
        hs = gobjects(1, numel(algs));
        for k = 1:numel(algs)
            y = L(:, s, k);
            if all(~isfinite(y)), continue; end
            lw = 1.6; if strcmp(algs{k}, 'AB-IRRT*'), lw = 3.2; end
            hs(k) = plot(ax, budgets, y, '-o', 'Color', alg_color(algs{k}), 'LineWidth', lw, ...
                'MarkerSize', 5, 'MarkerFaceColor', alg_color(algs{k}));
            % 成功集合随预算变化时，叠加“三档都成功”的配对子集均值（虚线）
            yp = Lp(:, s, k);
            if any(Suc(:, s, k) ~= Suc(1, s, k)) && all(isfinite(yp))
                plot(ax, budgets, yp, '--', 'Color', alg_color(algs{k}), ...
                    'LineWidth', max(1.0, lw * 0.7), 'HandleVisibility', 'off');
            end
        end
        set(ax, 'XTick', budgets, 'XTickLabelRotation', 0, 'XScale', 'log', 'FontSize', 9);
        xlim(ax, [min(budgets) * 0.8, max(budgets) * 1.25]);
        ylabel(ax, 'Mean path length'); xlabel(ax, 'Sampling budget (extensions)');
        title(ax, names{s}, 'FontSize', 12, 'FontWeight', 'bold'); grid(ax, 'on');
        if s == numel(scenes)
            hS = plot(ax, nan, nan, '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.6, ...
                'DisplayName', 'mean over successful runs');
            hD = plot(ax, nan, nan, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.2, ...
                'DisplayName', 'mean over runs successful at all budgets');
            keep = isgraphics(hs);
            lgd = legend(ax, [hs(keep), hS, hD], [algs(keep), ...
                {'mean over successful runs', 'mean over runs successful at all budgets'}], ...
                'FontSize', 8.5, 'Box', 'off');
            set(lgd, 'Units', 'normalized', 'Location', 'none', ...
                'Position', [0.772 0.24 0.220 0.50]);
        end
    end
    sgtitle(fig, 'Sensitivity to the sampling budget (2-D, 50 paired runs per budget)', ...
        'FontSize', 13.5, 'FontWeight', 'bold');
    save_fig(fig, outDir, 'fig_budget_2d');
    close(fig);
end
log_progress('=== make_extra_figures DONE ===');
end

function v = mean_or_nan(x)
if isempty(x), v = NaN; else, v = mean(x); end
end

function v = std_or_nan(x)
if numel(x) < 2, v = 0; else, v = std(x); end
end

function v = median_or_nan(x)
if isempty(x), v = NaN; else, v = median(x); end
end

function c = alg_color(name)
switch name
    case 'AB-IRRT*',      c = [0.85 0.08 0.08];
    case 'RRT*',          c = [0.25 0.25 0.28];
    case 'Informed-RRT*', c = [0.08 0.35 0.88];
    case 'RRT-Connect',   c = [0.05 0.62 0.28];
    case 'PRM*',          c = [0.55 0.12 0.72];
    case 'FMT*',          c = [0.95 0.55 0.05];
    case 'BIT*',          c = [0.05 0.60 0.72];
    case 'RRT#',          c = [0.88 0.25 0.62];
    case 'APF-only',      c = [0.55 0.55 0.05];
    otherwise,            c = [0 0 0];
end
end

function save_fig(fig, outDir, name)
% print + 显式纸面尺寸：保证导出图与屏幕版式等比（exportgraphics 会拉伸版式）。
w = fig.Position(3); h = fig.Position(4);
set(fig, 'PaperUnits', 'inches', 'PaperPositionMode', 'manual', ...
    'PaperPosition', [0 0 w / 96, h / 96]);
print(fig, fullfile(outDir, [name, '.png']), '-dpng', '-r200');
try
    print(fig, fullfile(outDir, [name, '.pdf']), '-dpdf');
catch
end
end
