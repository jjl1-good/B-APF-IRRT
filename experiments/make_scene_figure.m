function make_scene_figure()
%MAKE_SCENE_FIGURE 三个新增三维场景的“场景定义图”（只画几何，不画路径）。
%   替代原来三张“场景 + 四规划器路径”的图：路径已在九算法对比图里给出，
%   这里只交代障碍几何、坐标范围与起终点，避免同一批路径画两遍。
%   输出：figures/fig_scenes_3d_new.png/.pdf
%
%   字号/图宽比按“插入 docx 后 15.5 cm 宽”反推（有效字号 = 源字号 × 585.6 / 图宽px）：
%   图宽 1500 px + 刻度 20 pt ≈ 7.8 pt 有效，标题 24 pt ≈ 9.4 pt 有效。
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
wsRoot = fileparts(rootDir);
outDir = fullfile(rootDir, 'figures');
if ~exist(outDir, 'dir'), mkdir(outDir); end
addpath(fullfile(rootDir, 'algorithms_3d'));

modes = {'suspended', 'ring', 'overhang'};
titles = {'(a) suspended spheres', '(b) ring gallery', '(c) overhanging platforms'};

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [40 40 1500 540]);
for mi = 1:numel(modes)
    env = bair_env3d(modes{mi});
    fprintf('%s: cubes=%d cylinders=%d spheres=%d bounds=[%g %g %g %g %g %g]\n', ...
        modes{mi}, numel(env.cube.axisX), numel(env.cylinder.X), numel(env.sphere.X), env.bounds);
    ax = subplot(1, 3, mi); hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
    draw_env3d(ax, env);
    plot3(ax, env.start(1), env.start(2), env.start(3), 'o', 'MarkerSize', 12, ...
        'MarkerFaceColor', [0 0 0], 'MarkerEdgeColor', [1 1 1], 'LineWidth', 1.2);
    plot3(ax, env.goal(1), env.goal(2), env.goal(3), 'p', 'MarkerSize', 20, ...
        'MarkerFaceColor', [0.05 0.75 0.20], 'MarkerEdgeColor', [0 0 0], 'LineWidth', 1.2);
    axis(ax, [env.bounds(1:2), env.bounds(3:4), env.bounds(5:6)]);
    set(ax, 'SortMethod', 'childorder', 'FontSize', 20);
    xlabel(ax, 'x', 'FontSize', 20); ylabel(ax, 'y', 'FontSize', 20);
    zlabel(ax, 'z', 'FontSize', 20);
    set(ax, 'XTick', linspace(env.bounds(1), env.bounds(2), 3), ...
        'YTick', linspace(env.bounds(3), env.bounds(4), 3), ...
        'ZTick', linspace(env.bounds(5), env.bounds(6), 3));
    title(ax, titles{mi}, 'FontSize', 24, 'FontWeight', 'bold');
    view(ax, -37.5, 22);
    if mi == 1
        % 不用图例（会压住 z 刻度），直接在标记旁标注
        text(ax, env.start(1) - 12, env.start(2) + 22, env.start(3) + 6, 'start', ...
            'FontSize', 19, 'Color', [0 0 0], 'FontWeight', 'bold');
        text(ax, env.goal(1) + 10, env.goal(2) - 4, env.goal(3) - 16, 'goal', ...
            'FontSize', 19, 'Color', [0.05 0.55 0.15], 'FontWeight', 'bold');
    end
end

save_fig(fig, outDir, 'fig_scenes_3d_new');
close(fig);
fprintf('=== make_scene_figure DONE ===\n');
end

% ---------------- 局部工具（与 make_revision_figures.m 一致） ----------------
function draw_env3d(ax, env)
for k = 1:numel(env.cube.axisX)
    lo = [env.cube.axisX(k), env.cube.axisY(k), env.cube.axisZ(k)];
    ln = [env.cube.lengthx(k), env.cube.lengthy(k), env.cube.lengthz(k)];
    draw_cube(ax, lo, ln);
end
for k = 1:numel(env.cylinder.X)
    R = env.cylinder.radius(k); H = env.cylinder.lengthZ(k);
    [cx, cy, cz] = cylinder(R, 32);
    surf(ax, cx + env.cylinder.X(k), cy + env.cylinder.Y(k), cz * H + env.cylinder.Z(k), ...
        'FaceColor', [0.32 0.32 0.35], 'EdgeColor', 'none', 'FaceAlpha', 0.42);
end
for k = 1:numel(env.sphere.X)
    [sx, sy, sz] = sphere(28);
    R = env.sphere.radius(k);
    surf(ax, sx * R + env.sphere.X(k), sy * R + env.sphere.Y(k), sz * R + env.sphere.Z(k), ...
        'FaceColor', [0.40 0.40 0.43], 'EdgeColor', 'none', 'FaceAlpha', 0.42);
end
end

function draw_cube(ax, lo, ln)
P = [lo(1) lo(2) lo(3); lo(1) + ln(1) lo(2) lo(3); lo(1) + ln(1) lo(2) + ln(2) lo(3); lo(1) lo(2) + ln(2) lo(3); ...
     lo(1) lo(2) lo(3) + ln(3); lo(1) + ln(1) lo(2) lo(3) + ln(3); lo(1) + ln(1) lo(2) + ln(2) lo(3) + ln(3); lo(1) lo(2) + ln(2) lo(3) + ln(3)];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch(ax, 'Vertices', P, 'Faces', F, 'FaceColor', [0.32 0.32 0.35], ...
    'EdgeColor', [0.05 0.05 0.05], 'LineWidth', 0.5, 'FaceAlpha', 0.40);
end

function save_fig(fig, outDir, name)
w = fig.Position(3); h = fig.Position(4);
set(fig, 'PaperUnits', 'inches', 'PaperPositionMode', 'manual', ...
    'PaperPosition', [0 0 w / 96, h / 96]);
print(fig, fullfile(outDir, [name, '.png']), '-dpng', '-r200');
try
    print(fig, fullfile(outDir, [name, '.pdf']), '-dpdf');
catch
end
end
