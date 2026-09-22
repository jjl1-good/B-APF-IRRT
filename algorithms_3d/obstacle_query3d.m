function [d, nrm, id] = obstacle_query3d(p, env)
%OBSTACLE_QUERY3D 点到最近障碍物的距离与法向（三维）。
%
%   d   : 点 p 到最近障碍物表面的距离；p 位于障碍物内部时为 0。
%   nrm : 单位法向量，方向为“指向自由空间”（p 在外部时由表面指向 p；
%         p 在内部时指向最近的逃逸方向）。
%   id  : 最近图元编号，(类型-1)*1000+索引：1=立方体 2=圆柱 3=球。
%
% p 为 1×3 点；env 需包含字段：
%   cube     : axisX/axisY/axisZ（起点）与 lengthx/lengthy/lengthz
%   cylinder : X/Y/Z（底面圆心与高度起点）、radius、lengthZ
%   sphere   : X/Y/Z、radius
% 该函数与 collisionChecking3D.m 使用同一套图元定义，用于 APF 斥力、
% 三维绕行方向决策与路径最小间隙统计。

p = double(p(:)');
d = inf; nrm = [0 0 0]; id = 0;

% ---- 立方体 ----
for k = 1:numel(env.cube.axisX)
    lo = [env.cube.axisX(k), env.cube.axisY(k), env.cube.axisZ(k)];
    hi = lo + [env.cube.lengthx(k), env.cube.lengthy(k), env.cube.lengthz(k)];
    q = min(max(p, lo), hi);
    v = p - q; dist = norm(v);
    if dist > 1e-12
        if dist < d, d = dist; nrm = v / dist; id = k; end
    else
        dists = [p(1)-lo(1), hi(1)-p(1), p(2)-lo(2), hi(2)-p(2), p(3)-lo(3), hi(3)-p(3)];
        [m, which] = min(dists);
        if m < d
            d = max(m, 0); id = k;
            switch which
                case 1, nrm = [-1 0 0];
                case 2, nrm = [ 1 0 0];
                case 3, nrm = [0 -1 0];
                case 4, nrm = [0  1 0];
                case 5, nrm = [0 0 -1];
                otherwise, nrm = [0 0 1];
            end
        end
    end
end

% ---- 圆柱（轴向为 z，有限高度）----
for k = 1:numel(env.cylinder.X)
    cxy = [env.cylinder.X(k), env.cylinder.Y(k)];
    R = env.cylinder.radius(k);
    z0 = env.cylinder.Z(k); z1 = z0 + env.cylinder.lengthZ(k);
    rv = p(1:2) - cxy; rho = norm(rv);
    if rho < 1e-12, radialDir = [1 0 0]; else, radialDir = [rv(1)/rho, rv(2)/rho, 0]; end
    if p(3) >= z0 && p(3) <= z1
        if rho <= R
            % 内部：到最近表面（侧面/端盖）的距离
            dists = [R - rho, p(3) - z0, z1 - p(3)];
            [m, which] = min(dists);
            if m < d
                d = max(m, 0); id = 2000 + k;
                switch which
                    case 1, nrm = radialDir;            % 向侧面逃逸
                    case 2, nrm = [0 0 -1];             % 向下端盖逃逸
                    otherwise, nrm = [0 0 1];           % 向上端盖逃逸
                end
            end
        else
            if rho - R < d, d = rho - R; nrm = radialDir; id = 2000 + k; end
        end
    else
        zc = min(max(p(3), z0), z1);
        if rho <= R
            % 沿轴向的端盖上方/下方
            lat = 0; vert = abs(p(3) - zc);
            dist = vert;
            if dist < d
                d = dist; nrm = [0 0 sign(p(3) - zc)]; id = 2000 + k;
            end
        else
            % 边缘（rim）附近：到边缘圆的最近点
            rimPt = [cxy + R * radialDir(1:2), zc];
            v = p - rimPt; dist = norm(v);
            if dist < d && dist > 1e-12
                d = dist; nrm = v / dist; id = 2000 + k;
            end
        end
    end
end

% ---- 球 ----
for k = 1:numel(env.sphere.X)
    c = [env.sphere.X(k), env.sphere.Y(k), env.sphere.Z(k)];
    R = env.sphere.radius(k);
    v = p - c; rr = norm(v);
    if rr <= R
        if 0 < d, d = 0; id = 3000 + k; if rr < 1e-12, nrm = [1 0 0]; else, nrm = v / rr; end; end
    else
        if rr - R < d && rr > 1e-12
            d = rr - R; nrm = v / rr; id = 3000 + k;
        end
    end
end
end
