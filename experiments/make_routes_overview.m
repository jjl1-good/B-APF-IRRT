function make_routes_overview()
%MAKE_ROUTES_OVERVIEW 把二维/三维“八算法路径叠加对比”合并成一张正文用双面板图。
%
% 输出：修改稿/figures/fig_routes_overview.pdf + .eps + .tif
%       （(a) 三个二维环境，(b) 五个三维场景）
% 口径：只写矢量 PDF + EPS 与 600 dpi TIFF，不写 PNG。
% 说明：两张源图本身是宽幅多面板图，纵向堆叠后整体长宽比约 1.9，
%       与单张图占版面积相当，适合作为正文的“路径可视化”总览图。

here = fileparts(mfilename('fullpath'));
figdir = fullfile(here, '..', '..', '修改稿', 'figures');
A = imread(fullfile(figdir, 'fig2d_overlay_all.tif'));
B = imread(fullfile(figdir, 'fig3d_overlay_all.tif'));
if size(A, 3) == 1, A = repmat(A, 1, 1, 3); end
if size(B, 3) == 1, B = repmat(B, 1, 1, 3); end

W = max(size(A, 2), size(B, 2));
A = pad_white(A, W);
B = pad_white(B, W);
gap = round(0.025 * W);
canvas = uint8(255 * ones(size(A, 1) + gap + size(B, 1), W, 3));
canvas(1:size(A, 1), :, :) = A;
canvas(size(A, 1) + gap + (1:size(B, 1)), :, :) = B;

fw = 1800;
fh = round(fw * size(canvas, 1) / size(canvas, 2));
f = figure('Visible', 'off', 'Color', 'w', 'Units', 'pixels', 'Position', [80 80 fw fh]);
ax = axes(f, 'Position', [0 0 1 1]);
image(ax, canvas);
ax.XTick = []; ax.YTick = []; ax.Box = 'off'; axis(ax, 'image');
hold(ax, 'on'); ax.YDir = 'reverse';

lab(1) = annotation(f, 'textbox', [0.006, 0.955, 0.06, 0.04], 'String', '(a)', ...
                    'LineStyle', 'none', 'FontSize', 16, 'FontWeight', 'bold');
lab(2) = annotation(f, 'textbox', [0.006, 0.10, 0.06, 0.04], 'String', '(b)', ...
                    'LineStyle', 'none', 'FontSize', 16, 'FontWeight', 'bold');
export_figure(f, figdir, 'fig_routes_overview', struct('raster', true));
close(f);
fprintf('saved: fig_routes_overview(.pdf/.eps/.tif)  (%d x %d)\n', size(canvas, 2), size(canvas, 1));
end

function I = pad_white(I, W)
if size(I, 2) == W, return; end
pad = uint8(255 * ones(size(I, 1), W - size(I, 2), 3));
I = cat(2, I, pad);
end
