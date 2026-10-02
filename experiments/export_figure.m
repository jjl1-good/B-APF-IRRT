function export_figure(fig, outDir, name, opts)
%EXPORT_FIGURE 统一导出图件：矢量 PDF + EPS，以及供 Word 内嵌的高分辨率 TIFF。
%
% 口径（用户锁定）：
%   * 图件只提供矢量 PDF 与 EPS，**不再写 PNG**；
%   * 纯矢量面板保持矢量（PDF 用 ContentType 'vector'）；
%   * 含位图元素的面板（Gazebo 渲染帧、采样热图、路径叠加）以至少 600 dpi 内嵌该位图，
%     从而在印刷宽度下有效分辨率远高于 300 dpi；
%   * 另导一份 600 dpi TIFF 供 docx 内嵌，保证 Word 中印刷分辨率 ≥300 dpi。
%
% 用法：
%   export_figure(fig, outDir, 'fig_name');
%   export_figure(fig, outDir, 'fig_name', struct('raster', true, 'tif', true, 'vec', true));
%
% 选项：
%   vec     (true)  是否导出矢量 PDF / EPS（内容为半透明面片与海量点时改为 false 可控体积）
%   raster  (false) 该图是否含位图元素（决定 PDF 用 image 还是 vector 内容类型与内嵌 dpi）
%   tif     (true)  是否导出 600 dpi TIFF
%   dpi     (600)   位图内嵌与 TIFF 的分辨率

if nargin < 4, opts = struct(); end
if ~isfield(opts, 'vec'),    opts.vec = true;    end
if ~isfield(opts, 'raster'), opts.raster = false; end
if ~isfield(opts, 'tif'),    opts.tif = true;    end
if ~isfield(opts, 'dpi'),    opts.dpi = 600;     end

if ~exist(outDir, 'dir'), mkdir(outDir); end

% 明确纸面尺寸，保证导出图与屏幕版式等比（面板不被拉伸）
w = fig.Position(3); h = fig.Position(4);
try
    style_figures_for_review(fig);   % 若当前路径上没有该函数则跳过
catch
end
set(fig, 'PaperUnits', 'inches', 'PaperPositionMode', 'manual', ...
    'PaperPosition', [0 0 w / 96, h / 96]);

if opts.vec
    try
        if opts.raster
            print(fig, fullfile(outDir, [name, '.pdf']), '-dpdf', ...
                '-r600', '-opengl');
        else
            print(fig, fullfile(outDir, [name, '.pdf']), '-dpdf', ...
                '-painters');
        end
    catch ME
        warning('export_figure:pdf', 'PDF export failed for %s: %s', name, ME.message);
    end
    try
        if opts.raster
            print(fig, fullfile(outDir, [name, '.eps']), '-depsc', ...
                '-r600', '-opengl');
        else
            print(fig, fullfile(outDir, [name, '.eps']), '-depsc', '-painters');
        end
    catch ME
        warning('export_figure:eps', 'EPS export failed for %s: %s', name, ME.message);
    end
end

if opts.tif
    try
        print(fig, fullfile(outDir, [name, '.tif']), '-dtiff', ...
            sprintf('-r%d', opts.dpi), '-opengl');
    catch ME
        warning('export_figure:tif', 'TIFF export failed for %s: %s', name, ME.message);
    end
end
end
