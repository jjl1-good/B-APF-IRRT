function make_method_figure()
%MAKE_METHOD_FIGURE 方法示意图（两张，论文中用作 Figure 1 与 Figure 2）。
%   Figure 1  fig_framework.png : AB-IRRT* 两阶段总体框架（流程图）
%   Figure 2  fig_mechanism.png : (a) 三维决策平面内的绕行方向判据
%                                 (b) 由种子代价 c_p 打开的知情采样椭圆与偏置采样
%   字号按“插入 docx 后 15.5 cm 宽”反推：图宽 820 / 980 px，正文 11.5–14 pt，
%   等比缩到 15.5 cm 后仍在 8 pt 以上（MDPI 对图内字号的要求）。
%   输出：figures/fig_framework.png/.pdf、fig_mechanism.png/.pdf
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
wsRoot = fileparts(rootDir);
outDir = fullfile(rootDir, 'figures');
if ~exist(outDir, 'dir'), mkdir(outDir); end

C_L1 = [0.85 0.90 0.98];
C_L2 = [0.91 0.96 0.91];
C_FB = [0.97 0.93 0.86];
C_ED = [0.25 0.25 0.25];
C_TX = [0.08 0.08 0.08];

%% ==================== Figure 1：总体框架 ====================
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [40 40 820 600]);
ax = axes('Parent', fig, 'Position', [0 0 1 1], 'XLim', [0 82], 'YLim', [0 60], 'Visible', 'off');
hold(ax, 'on');

text(ax, 41, 57.2, 'AB-IRRT*: two-stage framework', 'FontSize', 15, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', C_TX);

box_round(ax, 3, 47.4, 76, 5.4, C_L1, C_ED);
text(ax, 41, 50.1, 'Stage 1: Bug-APF pre-planner', 'FontSize', 14, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', C_TX);

box_round(ax, 3, 37.6, 76, 4.4, C_FB, C_ED);
text(ax, 41, 39.8, 'stage one fails \rightarrow unseeded (cold) start', ...
    'FontSize', 11.5, 'HorizontalAlignment', 'center', 'Color', C_TX, 'Interpreter', 'tex');

box_round(ax, 3, 27.8, 76, 4.4, [0.96 0.96 0.96], C_ED);
text(ax, 41, 30.0, 'gated seed shortcutting', ...
    'FontSize', 13, 'HorizontalAlignment', 'center', 'Color', C_TX);

box_round(ax, 3, 17.0, 76, 5.4, C_L2, C_ED);
text(ax, 41, 19.7, 'Stage 2: enhanced Informed-RRT*', 'FontSize', 14, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', C_TX);

box_round(ax, 24, 7.0, 34, 4.4, [0.99 0.94 0.94], C_ED);
text(ax, 41, 9.2, 'output \sigma^*', 'FontSize', 14, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', [0.60 0.10 0.10], 'Interpreter', 'tex');

arrow(ax, 41, 47.4, 41, 42.0);
arrow(ax, 41, 37.6, 41, 32.2);
arrow(ax, 41, 27.8, 41, 22.4);
arrow(ax, 41, 17.0, 41, 11.4);

text(ax, 42.5, 44.8, 'path P', 'FontSize', 11, 'Color', [0.35 0.35 0.35], ...
    'HorizontalAlignment', 'left');
text(ax, 42.5, 35.0, 'shortcut seed', 'FontSize', 11, 'Color', [0.35 0.35 0.35], ...
    'HorizontalAlignment', 'left');
text(ax, 42.5, 25.0, 'c_{best} = c_p, sampling bias', 'FontSize', 11, ...
    'Color', [0.35 0.35 0.35], 'HorizontalAlignment', 'left', 'Interpreter', 'tex');

save_fig(fig, outDir, 'fig_framework');
close(fig);

%% ==================== Figure 2：关键几何机制 ====================
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [40 40 980 440]);
ax = axes('Parent', fig, 'Position', [0 0 1 1], 'XLim', [0 98], 'YLim', [0 44], 'Visible', 'off');
hold(ax, 'on');

%% ---- (a) 三维绕行方向 ----
text(ax, 2, 41.3, '(a) detour direction in 3-D', 'FontSize', 14, 'FontWeight', 'bold', ...
    'Color', C_TX);

cc = [15 16]; R = 8.0;                     % 障碍（圆）与半径
ang = linspace(0, 2 * pi, 100);
patch(ax, 'XData', cc(1) + R * cos(ang), 'YData', cc(2) + R * sin(ang), ...
    'FaceColor', [0.90 0.90 0.90], 'EdgeColor', [0.60 0.60 0.60], 'LineWidth', 1.1);
plot(ax, cc(1), cc(2), 'k+', 'MarkerSize', 9, 'LineWidth', 1.2);
text(ax, cc(1), cc(2) - 2.2, 'c', 'FontSize', 12, 'HorizontalAlignment', 'center');

phi = 55 * pi / 180;
x0 = cc(1) + R * cos(phi); y0 = cc(2) + R * sin(phi);
plot(ax, x0, y0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.10 0.10 0.10], ...
    'MarkerEdgeColor', 'none');
text(ax, x0 - 2.4, y0 + 0.4, 'x', 'FontSize', 13, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'right');

n = [cos(phi) sin(phi)];                   % 外法向
tg = [-sin(phi) cos(phi)];                 % 切向（逆时针）
F = [0.74 0.67]; F = F / norm(F);          % 吸引力方向（指向目标）
proj = F - (F * n') * n; proj = proj / norm(proj);

% 决策平面 span(n, t)
plane = [x0 y0; x0 + 9.0 * n(1), y0 + 9.0 * n(2); ...
    x0 + 9.0 * n(1) + 10 * tg(1), y0 + 9.0 * n(2) + 10 * tg(2); ...
    x0 + 10 * tg(1), y0 + 10 * tg(2)];
patch(ax, 'XData', plane(:, 1), 'YData', plane(:, 2), 'FaceColor', [0.80 0.85 0.97], ...
    'FaceAlpha', 0.55, 'EdgeColor', [0.35 0.45 0.75], 'LineWidth', 1.0);

arrow(ax, x0, y0, x0 + 9.0 * n(1), y0 + 9.0 * n(2), [0.15 0.15 0.15]);
text(ax, x0 + 9.6 * n(1), y0 + 8.6 * n(2), 'n', 'FontSize', 13, 'FontWeight', 'bold');
arrow(ax, x0, y0, x0 + 11 * F(1), y0 + 11 * F(2), [0.78 0.28 0.10]);
text(ax, x0 + 11.6 * F(1) + 0.6, y0 + 11.6 * F(2) - 1.6, 'F_{att}', 'FontSize', 12, ...
    'Color', [0.78 0.28 0.10], 'Interpreter', 'tex');
arrow(ax, x0, y0, x0 + 9.5 * proj(1), y0 + 9.5 * proj(2), [0.15 0.45 0.78]);
text(ax, x0 + 10.3 * proj(1), y0 + 10.3 * proj(2), 't', 'FontSize', 13, ...
    'FontWeight', 'bold', 'Color', [0.15 0.45 0.78]);

% 绕行方向：沿表面切向、由叉积符号定侧
arrow(ax, x0, y0, x0 + 10 * tg(1), y0 + 10 * tg(2), [0.55 0.10 0.55]);
text(ax, x0 + 11.5 * tg(1), y0 + 11.5 * tg(2) + 1.0, 'detour direction', 'FontSize', 11.5, ...
    'Color', [0.55 0.10 0.55], 'HorizontalAlignment', 'center');

plot(ax, [cc(1) x0], [cc(2) y0], '--', 'Color', [0.50 0.50 0.50], 'LineWidth', 0.9);
plot(ax, [cc(1) cc(1) + 19], [cc(2) cc(2) + 14], '--', 'Color', [0.50 0.50 0.50], 'LineWidth', 0.9);
plot(ax, cc(1) + 19, cc(2) + 14, 'p', 'MarkerSize', 12, 'MarkerFaceColor', [0.20 0.70 0.20], ...
    'MarkerEdgeColor', [0.10 0.45 0.10]);
text(ax, cc(1) + 18, cc(2) + 15.8, 'x_{goal}', 'FontSize', 12, 'Interpreter', 'tex');

text(ax, 2, 38.0, 'decision plane = span(n, t)', ...
    'FontSize', 11.5, 'Color', [0.25 0.30 0.55]);

%% ---- (b) 椭圆种子与偏置采样 ----
text(ax, 52, 41.3, '(b) ellipsoid seed and biased sampling', 'FontSize', 14, ...
    'FontWeight', 'bold', 'Color', C_TX);

cx = 73; cy = 25; a = 18.5; b = 11.5;
te = linspace(0, 2 * pi, 220);
plot(ax, cx + a * cos(te), cy + b * sin(te), '-', 'Color', [0.20 0.45 0.85], 'LineWidth', 1.6);
text(ax, cx, cy + b + 1.8, 'X_{ellipse}(c_p)', 'FontSize', 12, 'HorizontalAlignment', 'center', ...
    'Color', [0.20 0.45 0.85], 'Interpreter', 'tex');

xs_ = cx - a + 1.5; xg_ = cx + a - 1.5;
plot(ax, xs_, cy, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0 0 0], 'MarkerEdgeColor', 'none');
plot(ax, xg_, cy, 'p', 'MarkerSize', 13, 'MarkerFaceColor', [0.20 0.70 0.20], ...
    'MarkerEdgeColor', [0.10 0.45 0.10]);
text(ax, xs_ - 0.8, cy - 1.2, 'x_{start}', 'FontSize', 11, 'HorizontalAlignment', 'right', ...
    'Interpreter', 'tex');
text(ax, xg_ + 0.8, cy - 1.2, 'x_{goal}', 'FontSize', 11, 'HorizontalAlignment', 'left', ...
    'Interpreter', 'tex');

ts = linspace(0, 1, 60);
px_base = xs_ + (xg_ - xs_) * ts;
seed_y = cy + 6.6 * sin(pi * ts);
opt_y = cy + 2.2 * sin(pi * ts);

rng(7);
ns = 200;
u = rand(ns, 1);
sx = xs_ + (xg_ - xs_) * u + 1.1 * randn(ns, 1);
sy = cy + 6.6 * sin(pi * u) + 1.9 * randn(ns, 1);
inside = ((sx - cx) / a).^2 + ((sy - cy) / b).^2 <= 0.93;
plot(ax, sx(inside), sy(inside), '.', 'Color', [0.60 0.60 0.60], 'MarkerSize', 5);

h1 = plot(ax, px_base, seed_y, '--', 'Color', [0.90 0.55 0.05], 'LineWidth', 1.6);
h2 = plot(ax, px_base, opt_y, '-', 'Color', [0.75 0.10 0.10], 'LineWidth', 2.4);
h3 = plot(ax, nan, nan, '.', 'Color', [0.60 0.60 0.60], 'MarkerSize', 9);

lgd = legend(ax, [h1 h2 h3], {'pre-planned seed', 'stage-two path \sigma^*', 'biased samples'}, ...
    'FontSize', 10, 'Box', 'off', 'Location', 'none');
set(lgd, 'Units', 'normalized', 'Position', [0.545 0.09 0.22 0.11]);

save_fig(fig, outDir, 'fig_mechanism');
close(fig);

fprintf('=== make_method_figure DONE ===\n');
end

% ---------------- 局部工具 ----------------
function box_round(ax, x, y, w, h, fc, ec)
rectangle(ax, 'Position', [x y w h], 'Curvature', 0.14, 'FaceColor', fc, ...
    'EdgeColor', ec, 'LineWidth', 1.0);
end

function arrow(ax, x1, y1, x2, y2, col)
if nargin < 6, col = [0.25 0.25 0.25]; end
xl = get(ax, 'XLim'); yl = get(ax, 'YLim');
ann = annotation('arrow', [(x1 - xl(1)) (x2 - xl(1))] / diff(xl), ...
    [(y1 - yl(1)) (y2 - yl(1))] / diff(yl));
set(ann, 'Color', col, 'LineWidth', 1.4, 'HeadStyle', 'vback2', ...
    'HeadLength', 7, 'HeadWidth', 7);
end

function save_fig(fig, outDir, name)
% print + 显式纸面尺寸：保证导出图与屏幕版式等比（不要用 exportgraphics）。
w = fig.Position(3); h = fig.Position(4);
set(fig, 'PaperUnits', 'inches', 'PaperPositionMode', 'manual', ...
    'PaperPosition', [0 0 w / 96, h / 96]);
print(fig, fullfile(outDir, [name, '.png']), '-dpng', '-r200');
try
    print(fig, fullfile(outDir, [name, '.pdf']), '-dpdf');
catch
end
end
