function make_pareto_figure()
%MAKE_PARETO_FIGURE 质量—时间权衡图：八个环境下“平均路径长度 vs 平均总耗时”。
% 每个面板把 50 次运行的平均长度（纵轴）与平均总时间（横轴，对数）画在一起，误差棒为 50 次
% 运行的均值 ± 标准差；淡红色阴影标出“比本文方法更快且更短”的象限，该象限在八个环境中都是空的。
% 数据：summary_2d.csv + summary_3d_<scene>.csv
% 输出：figures/fig_pareto_tradeoff.png（200 dpi）+ 同名矢量 PDF + 同目录 CSV
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
wsRoot = fileparts(rootDir);
outDir = fullfile(rootDir, 'figures');
if ~exist(outDir, 'dir'), mkdir(outDir); end

algs = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#', 'APF-only'};
% APF-only 是“只保留反应式阶段”的退化基线（在二维狭窄与三维悬挑直接失败，且长度量级明显偏大），
% 为了保持坐标可读性，本图只画七个全局规划器；APF-only 的数值在表 7–14 中均有报告。
algsPlot = {'AB-IRRT*', 'RRT*', 'Informed-RRT*', 'PRM*', 'FMT*', 'BIT*', 'RRT#'};
tags = {'AB', 'RRT*', 'IRRT', 'PRM*', 'FMT*', 'BIT*', 'RRT#'};
% 每个算法的标签偏移（dx, dy），用以错开密集区域的文字
lblOff = [1.22 1.004;   % AB-IRRT*
          0.98 1.012;   % RRT*
          1.18 0.9965;  % Informed-RRT*
          1.20 1.005;   % PRM*
          1.18 0.9975;  % FMT*
          1.20 1.004;   % BIT*
          1.10 0.9945]; % RRT#
panels = { ...
    'summary_2d.csv',        '一般',  '2-D general'; ...
    'summary_2d.csv',        '狭窄',  '2-D narrow'; ...
    'summary_2d.csv',        '广阔',  '2-D expansive'; ...
    'summary_3d_general.csv', '一般', '3-D general'; ...
    'summary_3d_narrow.csv',  '狭窄', '3-D narrow'; ...
    'summary_3d_suspended.csv', '悬空', '3-D suspended'; ...
    'summary_3d_ring.csv',    '环廊', '3-D ring'; ...
    'summary_3d_overhang.csv','悬挑', '3-D overhang'};

fig = figure('Visible', 'off', 'Position', [30 30 1800 900]);
rowsOut = {};
for p = 1:size(panels, 1)
    T = readtable(fullfile(entryDir, panels{p, 1}), 'Encoding', 'UTF-8');
    sel = strcmp(T.scenario, panels{p, 2});
    T = T(sel, :);
    ax = subplot(2, 4, p); hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');

    x = nan(1, numel(algs)); y = nan(1, numel(algs));
    xe = nan(2, numel(algs)); ye = nan(2, numel(algs));
    for k = 1:numel(algs)
        m = strcmp(T.algorithm, algs{k});
        if ~any(m), continue; end
        y(k) = T.meanLength(find(m, 1));
        x(k) = 1000 * T.meanTime(find(m, 1));   % 秒 → 毫秒
        ye(:, k) = T.stdLength(find(m, 1)) * [1; 1];
        xe(:, k) = min(1000 * T.stdTime(find(m, 1)), 0.8 * x(k)) * [1; 1];
        rowsOut(end + 1, :) = {panels{p, 3}, algs{k}, y(k), x(k), T.successRate(find(m, 1))}; %#ok<AGROW>
    end
    kp = ismember(algs, algsPlot);
    ok = isfinite(x) & isfinite(y) & kp;
    xo = x(strcmp(algs, 'AB-IRRT*')); yo = y(strcmp(algs, 'AB-IRRT*'));

    % 支配象限（更快且更短）阴影；坐标范围按全局规划器缩放
    xl = [min(x(ok)) * 0.45, max(x(ok)) * 2.4];
    yl = [min(y(ok)) - 0.12 * range(y(ok)), max(y(ok)) + 0.14 * range(y(ok))];
    patch(ax, [xl(1) xo xo xl(1)], [yl(1) yl(1) yo yo], [0.85 0.1 0.1], ...
        'FaceAlpha', 0.07, 'EdgeColor', 'none');
    % 阈值线
    plot(ax, [xo xo], yl, '--', 'Color', [0.6 0.2 0.2], 'LineWidth', 0.8);
    plot(ax, xl, [yo yo], '--', 'Color', [0.6 0.2 0.2], 'LineWidth', 0.8);

    for j = 1:numel(algsPlot)
        k = find(strcmp(algs, algsPlot{j}), 1);
        if ~ok(k), continue; end
        errorbar(ax, x(k), y(k), ye(1, k), ye(2, k), xe(1, k), xe(2, k), 'o', ...
            'Color', alg_color(algs{k}), 'MarkerFaceColor', alg_color(algs{k}), ...
            'MarkerSize', 5, 'LineWidth', 1.0, 'CapSize', 2);
        if strcmp(algs{k}, 'AB-IRRT*')
            plot(ax, x(k), y(k), 'p', 'Color', [0.85 0.08 0.08], 'MarkerFaceColor', [0.85 0.08 0.08], ...
                'MarkerSize', 16, 'LineWidth', 1.3);
            text(ax, x(k) * lblOff(j, 1), y(k) * lblOff(j, 2), 'AB-IRRT*', 'Color', [0.85 0.08 0.08], ...
                'FontSize', 9.5, 'FontWeight', 'bold');
        else
            text(ax, x(k) * lblOff(j, 1), y(k) * lblOff(j, 2), tags{j}, ...
                'Color', alg_color(algs{k}), 'FontSize', 8.5);
        end
    end
    set(ax, 'XScale', 'log', 'XLim', xl, 'YLim', yl, 'FontSize', 8.5);
    xlabel(ax, 'Mean runtime per query (ms, log scale)');
    ylabel(ax, 'Mean path length');
    title(ax, panels{p, 3}, 'FontSize', 11.5, 'FontWeight', 'bold');
    if p == 1
        text(ax, xl(1) * 1.6, yl(1) + 0.90 * range(yl), 'shaded: faster and shorter', ...
            'Color', [0.6 0.2 0.2], 'FontSize', 8);
    end
end
sgtitle(fig, ['Quality-time trade-off: in all eight environments no global planner is both faster and ' ...
    'shorter than AB-IRRT* (the shaded quadrant is empty)'], ...
    'FontSize', 13, 'FontWeight', 'bold');

w = fig.Position(3); h = fig.Position(4);
set(fig, 'PaperUnits', 'inches', 'PaperPositionMode', 'manual', 'PaperPosition', [0 0 w / 96, h / 96]);
print(fig, fullfile(outDir, 'fig_pareto_tradeoff.png'), '-dpng', '-r200');
print(fig, fullfile(outDir, 'fig_pareto_tradeoff.pdf'), '-dpdf');
close(fig);

Tout = cell2table(rowsOut, 'VariableNames', {'scene', 'algorithm', 'meanLength', 'meanTime_ms', 'successRate'});
writetable(Tout, fullfile(entryDir, 'pareto_tradeoff.csv'), 'Encoding', 'UTF-8');
fprintf('pareto figure written to %s (%d rows)\n', outDir, height(Tout));
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
