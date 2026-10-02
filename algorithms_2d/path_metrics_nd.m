function m = path_metrics_nd(path, queryFcn, opts)
%PATH_METRICS_ND 路径质量指标（长度、平滑度、最小间隙、航点数）。
%
%   m = path_metrics_nd(path, queryFcn)
%   m = path_metrics_nd(path, queryFcn, opts)
%
% path     : K×d 路径点矩阵（d 为 2 或 3）。
% queryFcn : 距离查询函数句柄，queryFcn(pts) 返回 N×1 的最小障碍距离；
%            二维用 @(q) clear_query2d(q, env)，三维用 @(q) clear_query3d(q, env)，
%            或直接传入 obstacle_query2d/3d 的包装句柄。
% opts.sampleStep : 间隙采样步长（默认 0.5，与碰撞检测分辨率一致）。
%
% 输出 m 的字段：
%   length      总长度
%   turnSum     相邻线段转角之和（弧度），反映路径曲折程度
%   turnMean    平均转角
%   turnMax     最大单点转角
%   minClear    路径上的最小障碍距离（碰撞安全裕量的量化指标）
%   waypoints   路径点数
%   straightness 直线度 = 起终点直线距离/总长度（1 为完全笔直）

if nargin < 3, opts = struct; end
if ~isfield(opts, 'sampleStep'), opts.sampleStep = 0.5; end

path = double(path);
m = struct('length', inf, 'turnSum', NaN, 'turnMean', NaN, 'turnMax', NaN, ...
    'minClear', NaN, 'waypoints', size(path, 1), 'straightness', NaN);
if size(path, 1) < 2, return; end

% ---- 长度 ----
seg = diff(path);
segLen = sqrt(sum(seg.^2, 2));
m.length = sum(segLen);

% ---- 平滑度 ----
tol = 1e-9;
keep = segLen > tol;
S = seg(keep, :); L = segLen(keep);
if size(S, 1) >= 2
    S = S ./ L;                       % 单位方向
    cosv = sum(S(1:end-1,:) .* S(2:end,:), 2);
    ang = acos(min(max(cosv, -1), 1));
    m.turnSum = sum(ang);
    m.turnMean = mean(ang);
    m.turnMax = max(ang);
end

% ---- 最小间隙（沿路径等步长采样）----
pts = sample_polyline(path, opts.sampleStep);
if ~isempty(queryFcn) && ~isempty(pts)
    dq = queryFcn(pts);
    m.minClear = min(dq(:));
end

% ---- 直线度 ----
sl = norm(path(end,:) - path(1,:));
if m.length > 0, m.straightness = sl / m.length; end
end

function pts = sample_polyline(path, step)
% 沿折线等距采样（含端点），用于间隙评估。
segLen = sqrt(sum(diff(path).^2, 2));
total = sum(segLen);
if total <= 0, pts = path(1, :); return; end
n = max(2, ceil(total / step) + 1);
s = linspace(0, total, n);
pts = zeros(n, size(path, 2));
cum = [0; cumsum(segLen)];
for i = 1:n
    k = find(cum <= s(i), 1, 'last');
    k = min(max(k, 1), size(path, 1) - 1);
    t = (s(i) - cum(k)) / max(segLen(k), eps);
    pts(i, :) = path(k, :) + t * (path(k+1, :) - path(k, :));
end
end
