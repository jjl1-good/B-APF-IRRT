function feasible = collisionChecking3D(p1, p2, env, margin)
%COLLISIONCHECKING3D 精确线段-障碍物碰撞检测（解析法，无采样漏检）。
%
%   feasible = collisionChecking3D(p1, p2, env)
%   feasible = collisionChecking3D(p1, p2, env, margin)
%
% 返回 true 表示线段 (p1,p2) 无碰撞（与原有调用约定一致）。
%
% 相对原实现（按 0.5 分辨率逐点采样）的改进：
%   1) 立方体：平板（slab）解析检测，薄/斜穿线段不会因采样间隔漏检；
%   2) 圆柱体：侧面二次方程求交 + 端盖区间检查，支持任意方向线段；
%   3) 球体：二次方程解析求交；
%   4) margin：可选安全裕度，等效于把障碍物膨胀 margin（默认 0），
%      供安全距离检验与局部规划器使用。
% 该实现是论文“三维碰撞检测”一节所描述的精确判定，保证算法安全性
% 分析（R3-2）与实现一致。

if nargin < 4 || isempty(margin), margin = 0; end
feasible = false;
if nargin < 3 || numel(p1) < 3 || numel(p2) < 3, return; end
p1 = double(p1(1:3)); p2 = double(p2(1:3));
if ~inside3(p1, env.bounds) || ~inside3(p2, env.bounds), return; end
d = p2 - p1;

% ---- 立方体（含膨胀）----
for k = 1:numel(env.cube.axisX)
    lo = [env.cube.axisX(k), env.cube.axisY(k), env.cube.axisZ(k)] - margin;
    hi = lo + [env.cube.lengthx(k), env.cube.lengthy(k), env.cube.lengthz(k)] + 2 * margin;
    if segment_hits_aabb(p1, d, lo, hi), return; end
end

% ---- 圆柱体（含膨胀）----
for k = 1:numel(env.cylinder.X)
    cxy = [env.cylinder.X(k), env.cylinder.Y(k)];
    R = env.cylinder.radius(k) + margin;
    z0 = env.cylinder.Z(k) - margin;
    z1 = env.cylinder.Z(k) + env.cylinder.lengthZ(k) + margin;
    if segment_hits_cylinder(p1, d, cxy, R, z0, z1), return; end
end

% ---- 球体（含膨胀）----
for k = 1:numel(env.sphere.X)
    c = [env.sphere.X(k), env.sphere.Y(k), env.sphere.Z(k)];
    R = env.sphere.radius(k) + margin;
    if segment_hits_sphere(p1, d, c, R), return; end
end

feasible = true;
end

function hit = segment_hits_aabb(p0, d, lo, hi)
% 线段与轴对齐包围盒的 slab 解析求交（边界接触计为相交）。
tmin = 0; tmax = 1; hit = false;
for i = 1:3
    if abs(d(i)) < 1e-12
        if p0(i) < lo(i) - 1e-12 || p0(i) > hi(i) + 1e-12, return; end
    else
        t1 = (lo(i) - p0(i)) / d(i);
        t2 = (hi(i) - p0(i)) / d(i);
        if t1 > t2, tmp = t1; t1 = t2; t2 = tmp; end
        if t1 > tmin, tmin = t1; end
        if t2 < tmax, tmax = t2; end
        if tmin > tmax + 1e-12, return; end
    end
end
hit = true;
end

function hit = segment_hits_sphere(p0, d, c, R)
% 线段与球体解析求交。
m = p0 - c;
a = dot(d, d);
if a < 1e-18
    hit = dot(m, m) <= R^2;
    return;
end
b = 2 * dot(m, d);
cc = dot(m, m) - R^2;
disc = b^2 - 4 * a * cc;
if disc < 0, hit = false; return; end
sq = sqrt(disc);
t1 = (-b - sq) / (2 * a);
t2 = (-b + sq) / (2 * a);
hit = (t2 >= -1e-12) && (t1 <= 1 + 1e-12);
end

function hit = segment_hits_cylinder(p0, d, cxy, R, z0, z1)
% 线段与有限圆柱（轴向为 z）解析求交：侧面二次方程与端盖区间联合判定。
a = d(1)^2 + d(2)^2;
m = p0(1:2) - cxy;
if a < 1e-18
    % 竖直或静止线段：径向为常数
    if norm(m) > R + 1e-12, hit = false; return; end
    zlo = min(p0(3), p0(3) + d(3)); zhi = max(p0(3), p0(3) + d(3));
    hit = (zhi >= z0 - 1e-12) && (zlo <= z1 + 1e-12);
    return;
end
b = 2 * dot(m, d(1:2));
cc = dot(m, m) - R^2;
disc = b^2 - 4 * a * cc;
if disc < 0, hit = false; return; end
sq = sqrt(disc);
ta = max((-b - sq) / (2 * a), 0);
tb = min((-b + sq) / (2 * a), 1);
if ta > tb + 1e-12, hit = false; return; end
za = p0(3) + d(3) * ta;
zb = p0(3) + d(3) * tb;
if d(3) >= 0, zlo = za; zhi = zb; else, zlo = zb; zhi = za; end
hit = (zhi >= z0 - 1e-12) && (zlo <= z1 + 1e-12);
end

function ok = inside3(p, b)
ok = p(1) >= b(1) && p(1) <= b(2) && p(2) >= b(3) && p(2) <= b(4) && ...
     p(3) >= b(5) && p(3) <= b(6);
end
