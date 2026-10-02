function style_figures_for_review(fig)
%STYLE_FIGURES_FOR_REVIEW 统一提升数据图的可读性（审稿意见：图件不清晰）。
%
%   只调整字号、刻度方向、网格、线宽与标记大小，不修改坐标范围、颜色映射
%   与版式，因此在既有 PaperPosition 导出尺寸下不会改变图像布局。
%   在 print(...) 之前调用：
%       style_figures_for_review(fig);
%       set(fig, 'PaperUnits', 'inches', ...);
%       print(fig, filename, '-dpng', '-r200');
%
%   改动口径（可在回复信中引用）：
%     * 轴/刻度字号 +2 pt，并夹在 10.5-13 pt；
%     * 图例字号 +1.5 pt（9.5-12 pt），去掉图例边框；
%     * 色标字号 +1.5 pt（>=10 pt）；
%     * 刻度朝外、外侧加框、打开主网格（GridAlpha 0.15）；
%     * 曲线线宽下限 1.8 pt（点标记不下调），散点标记下限 5 pt。

if nargin < 1 || isempty(fig), fig = gcf; end

axesList = findall(fig, 'Type', 'axes');
for k = 1:numel(axesList)
    a = axesList(k);
    tag = get(a, 'Tag');
    if strcmp(tag, 'legend') || strcmp(tag, 'colorbar'), continue; end

    fs = a.FontSize;
    if isempty(fs) || ~isfinite(fs), fs = 9; end
    a.FontSize = max(10.5, min(13, fs + 2));

    % 面板标题不随轴字号一起放大，避免与 figure 级总标题重叠
    tObj = a.Title;
    if ~isempty(strtrim(strjoin(string(tObj.String), ' ')))
        cf = tObj.FontSize;
        if isempty(cf) || ~isfinite(cf) || cf <= 0
            tObj.FontSize = 11.5;
        else
            tObj.FontSize = max(9, min(11.5, cf));
        end
    end

    if isempty(a.LineWidth) || ~isfinite(a.LineWidth), a.LineWidth = 0.5; end
    a.LineWidth = max(a.LineWidth, 0.9);
    a.TickDir = 'out';
    a.Box = 'on';
    a.Layer = 'top';
    % 顶部留白：多面板图里避免面板标题与 figure 级总标题相撞
    if numel(axesList) > 1 && a.Position(4) > 0.25
        pp = a.Position;
        pp(4) = pp(4) * 0.93;
        a.Position = pp;
    end
    try
        grid(a, 'on');
        a.GridAlpha = 0.15;
        a.MinorGridAlpha = 0.08;
    catch
        % 极坐标等不支持网格的坐标轴，忽略
    end

    hs = findall(a, 'Type', 'line');
    for i = 1:numel(hs)
        h = hs(i);
        if strcmp(h.LineStyle, 'none')
            if ~isempty(h.Marker) && ~strcmp(h.Marker, 'none') && h.MarkerSize < 5
                h.MarkerSize = 5;
            end
        elseif h.LineWidth < 1.8
            h.LineWidth = 1.8;
        end
    end
end

lgList = findall(fig, 'Type', 'Legend');
for k = 1:numel(lgList)
    fs = lgList(k).FontSize;
    if isempty(fs) || ~isfinite(fs), fs = 9; end
    lgList(k).FontSize = max(9.5, min(12, fs + 1.5));
    lgList(k).Box = 'off';
end

cbList = findall(fig, 'Type', 'ColorBar');
for k = 1:numel(cbList)
    fs = cbList(k).FontSize;
    if isempty(fs) || ~isfinite(fs), fs = 9; end
    cbList(k).FontSize = max(10, min(12.5, fs + 1.5));
end

% figure 级总标题（sgtitle / annotation）略减，避免与面板标题相撞
txtList = findall(fig, 'Type', 'text');
for k = 1:numel(txtList)
    t = txtList(k);
    par = t.Parent;
    if ~isempty(par) && isprop(par, 'Type') && strcmp(par.Type, 'figure')
        fs = t.FontSize;
        if ~isempty(fs) && isfinite(fs) && fs >= 13
            t.FontSize = fs - 1.5;
        end
    end
end
end
