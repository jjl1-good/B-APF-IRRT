function feasible = collisionChecking(point1, point2, squareObstacles, roundObstacles)
%COLLISIONCHECKING 检查二维线段是否与障碍物相交。
%
%   feasible = collisionChecking(point1, point2, squareObstacles,
%                                roundObstacles)
%
% 返回值遵循当前二维算法中的调用约定：
%   feasible == 1  表示线段无碰撞（可行）
%   feasible == 0  表示线段与障碍物相交（不可行）
%
% point1/point2      为 [x y] 点；
% squareObstacles   为 N×4 矩阵，每行为 [xmin ymin width height]；
% roundObstacles    为 M×3 矩阵，每行为 [xc yc radius]。

% 默认认为路径可行；参数异常时返回不可行，避免误放行。
feasible = false;

if nargin < 2 || numel(point1) < 2 || numel(point2) < 2
    return;
end
if nargin < 3 || isempty(squareObstacles)
    squareObstacles = zeros(0, 4);
end
if nargin < 4 || isempty(roundObstacles)
    roundObstacles = zeros(0, 3);
end

point1 = double(point1(1:2));
point2 = double(point2(1:2));
squareObstacles = double(squareObstacles);
roundObstacles = double(roundObstacles);

if size(squareObstacles, 2) < 4 || size(roundObstacles, 2) < 3 || ...
        any(~isfinite([point1 point2]))
    return;
end

% 线段与矩形（含边界）相交即视为碰撞。
for k = 1:size(squareObstacles, 1)
    rect = squareObstacles(k, 1:4);
    if any(~isfinite(rect)) || rect(3) < 0 || rect(4) < 0
        return;
    end
    xmin = rect(1);
    xmax = rect(1) + rect(3);
    ymin = rect(2);
    ymax = rect(2) + rect(4);
    if segmentIntersectsRectangle(point1, point2, xmin, xmax, ymin, ymax)
        return;
    end
end

% 线段到圆心的最短距离不大于半径即视为碰撞。
for k = 1:size(roundObstacles, 1)
    circle = roundObstacles(k, 1:3);
    if any(~isfinite(circle)) || circle(3) < 0
        return;
    end
    if pointSegmentDistanceSquared(point1, point2, circle(1:2)) <= circle(3)^2
        return;
    end
end

feasible = true;
end

function intersects = segmentIntersectsRectangle(p0, p1, xmin, xmax, ymin, ymax)
% Liang-Barsky 线段裁剪算法；边界接触也计为相交。
dx = p1(1) - p0(1);
dy = p1(2) - p0(2);
t0 = 0;
t1 = 1;

[ok, t0, t1] = clipTest(-dx, p0(1) - xmin, t0, t1);
if ~ok
    intersects = false;
    return;
end
[ok, t0, t1] = clipTest(dx, xmax - p0(1), t0, t1);
if ~ok
    intersects = false;
    return;
end
[ok, t0, t1] = clipTest(-dy, p0(2) - ymin, t0, t1);
if ~ok
    intersects = false;
    return;
end
[ok, t0, t1] = clipTest(dy, ymax - p0(2), t0, t1);
intersects = ok && t0 <= t1;
end

function [ok, t0, t1] = clipTest(p, q, t0, t1)
tol = 1e-12;
if abs(p) <= tol
    ok = q >= -tol;
    return;
end

r = q / p;
if p < 0
    if r > t1
        ok = false;
        return;
    end
    if r > t0
        t0 = r;
    end
else
    if r < t0
        ok = false;
        return;
    end
    if r < t1
        t1 = r;
    end
end
ok = true;
end

function distanceSquared = pointSegmentDistanceSquared(p0, p1, point)
segment = p1 - p0;
segmentLengthSquared = dot(segment, segment);
if segmentLengthSquared <= eps
    closest = p0;
else
    projection = dot(point - p0, segment) / segmentLengthSquared;
    projection = max(0, min(1, projection));
    closest = p0 + projection * segment;
end
delta = point - closest;
distanceSquared = dot(delta, delta);
end
