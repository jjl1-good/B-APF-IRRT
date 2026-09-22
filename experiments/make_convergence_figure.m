function make_convergence_figure()
%MAKE_CONVERGENCE_FIGURE 代价—采样数收敛曲线：八个算法在二维三个环境下的平均路径长度随预算变化。
% 数据：results_2d_b100/b250/b500/b750/b1000.csv + results_2d_v2.csv（= 2000 次扩展）。
% 输出：figures/fig_convergence_2d.png（只导 PNG）

entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
wsRoot = fileparts(rootDir);
outDir = fullfile(rootDir, 'figures');
addpath(entryDir); addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));

budgets = [100 250 500 750 1000 2000];
files = {'results_2d_b100.csv', 'results_2d_b250.csv', 'results_2d_b500.csv', ...
    'results_2d_b750.csv', 'results_2d_b1000.csv', 'results_2d_v2.csv'};
algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
scenes = {'一般', '狭窄', '广阔'};
names = {'General 2-D (150 x 150)', 'Narrow-passage 2-D', 'Expansive 2-D (300 x 300)'};

M = nan(numel(algs), numel(budgets), numel(scenes));   % 均值长度
S = nan(numel(algs), numel(budgets), numel(scenes));   % 成功率
for b = 1:numel(budgets)
    f = fullfile(entryDir, files{b});
    if ~exist(f, 'file'), continue; end
    T = readtable(f, 'Encoding', 'UTF-8');
    for si = 1:numel(scenes)
        for ai = 1:numel(algs)
            sel = strcmp(T.scenario, scenes{si}) & strcmp(T.algorithm, algs{ai});
            if ~any(sel), continue; end
            succ = T.success(sel);
            L = T.length(sel);
            good = succ == 1 & isfinite(L);
            if any(good)
                M(ai, b, si) = mean(L(good));
            end
            S(ai, b, si) = 100 * mean(succ);
        end
    end
end

fig = figure('Visible', 'off', 'Position', [40 40 1500 700]);
tl = tiledlayout(fig, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
axTop = gobjects(1, numel(scenes));
for si = 1:numel(scenes)
    ax = nexttile(tl, [2 1]); hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
    axTop(si) = ax;
    tgt = min(M(:, end, si), [], 'omitnan');
    yline(ax, tgt * 1.01, ':', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.2);
    text(ax, budgets(1), tgt * 1.01, ' best + 1 %', 'FontSize', 9, 'Color', [0.2 0.2 0.2]);
    hs = gobjects(1, numel(algs));
    for ai = 1:numel(algs)
        y = squeeze(M(ai, :, si));
        ok = isfinite(y);
        if ~any(ok), continue; end
        hs(ai) = plot(ax, budgets(ok), y(ok), '-o', 'Color', alg_color(algs{ai}), 'LineWidth', 1.8, ...
            'MarkerSize', 5, 'MarkerFaceColor', alg_color(algs{ai}));
    end
    if si == 1
        keep = isgraphics(hs);
        lgH = hs(keep); lgT = algs(keep);
    end
    set(ax, 'XScale', 'log', 'FontSize', 10);
    xlim(ax, [0.8 * budgets(1), 1.3 * budgets(end)]);
    if si == 1, ylabel(ax, 'Mean path length (successful runs)'); end
    title(ax, names{si}, 'FontSize', 12);
end
% 下排：同一横轴上的成功率——低预算下的均值只在成功运行上统计，
% 这一排直接把“预算影响成功率”展示出来，避免把构成效应当成算法退化。
for si = 1:numel(scenes)
    ax2 = nexttile(tl); hold(ax2, 'on'); box(ax2, 'on'); grid(ax2, 'on');
    for ai = 1:numel(algs)
        y = squeeze(S(ai, :, si));
        ok = isfinite(y);
        if ~any(ok), continue; end
        plot(ax2, budgets(ok), y(ok), '-o', 'Color', alg_color(algs{ai}), 'LineWidth', 1.5, ...
            'MarkerSize', 4, 'MarkerFaceColor', alg_color(algs{ai}));
    end
    set(ax2, 'XScale', 'log', 'FontSize', 10);
    xlim(ax2, [0.8 * budgets(1), 1.3 * budgets(end)]);
    ylim(ax2, [0 105]);
    xlabel(ax2, 'Sampling / extension budget N');
    if si == 1, ylabel(ax2, 'Success rate (%)'); end
end
title(tl, 'Convergence versus sampling budget (mean over 50 paired runs per point)', ...
    'FontSize', 13, 'FontWeight', 'bold');
lgd = legend(axTop(1), lgH, lgT, 'Orientation', 'horizontal', 'NumColumns', 5, 'FontSize', 10);
lgd.Layout.Tile = 'south';

w = fig.Position(3); hh = fig.Position(4);
set(fig, 'PaperUnits', 'inches', 'PaperPositionMode', 'manual', 'PaperPosition', [0 0 w / 96, hh / 96]);
print(fig, fullfile(outDir, 'fig_convergence_2d.png'), '-dpng', '-r200');
print(fig, fullfile(outDir, 'fig_convergence_2d.pdf'), '-dpdf');
close(fig);
fprintf('convergence figure written to %s\n', outDir);

% 数值汇总
fprintf('\n%-15s' , 'algorithm'); fprintf('%3s', ''); 
for b = 1:numel(budgets), fprintf(' N=%-6d', budgets(b)); end
fprintf('\n');
for si = 1:numel(scenes)
    fprintf('-- %s --\n', scenes{si});
    for ai = 1:numel(algs)
        fprintf('%-15s', algs{ai});
        fprintf(' %7.1f', squeeze(M(ai, :, si)));
        fprintf('\n');
    end
end
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
    otherwise,            c = [0.55 0.55 0.05];
end
end
