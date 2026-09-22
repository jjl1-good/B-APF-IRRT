function [d, nrm, id] = obstacle_query2d(p, env)
%OBSTACLE_QUERY2D 点到最近障碍物的距离与法向（二维）。
%
%   d   : 点 p 到最近障碍物表面的距离；p 位于障碍物内部时为 0。
%   nrm : 单位法向量，方向为“指向自由空间”（p 在外部时由表面指向 p；
%         p 在内部时指向最近的逃逸面）。
%   id  : 最近图元的编号，编码为 (类型-1)*1000+索引，类型 1=矩形、2=圆。
%
% p 为 1×2 点；env 需包含字段 squareAll（N×4，[xmin ymin w h]）与
% round（M×3，[cx cy r]）。该函数用于 APF 斥力、路径最小间隙等指标，
% 保证与 collisionChecking.m 使用同一套几何定义。

p = double(p(:)');
d = inf; nrm = [0 0]; id = 0;

for k = 1:size(env.squareAll, 1)
    r = env.squareAll(k, :);
    xmin = r(1); xmax = r(1) + r(3); ymin = r(2); ymax = r(2) + r(4);
    qx = min(max(p(1), xmin), xmax);
    qy = min(max(p(2), ymin), ymax);
    v = p - [qx qy];
    dist = norm(v);
    if dist > 1e-12
        if dist < d
            d = dist; nrm = v / dist; id = k;
        end
    else
        % 点在矩形内部：到最近面的距离，法向指向该面外侧（逃逸方向）。
        dxl = p(1) - xmin; dxr = xmax - p(1);
        dyl = p(2) - ymin; dyr = ymax - p(2);
        [m, which] = min([dxl dxr dyl dyr]);
        if m < d
            d = max(m, 0); id = k;
            switch which
                case 1, nrm = [-1 0];
                case 2, nrm = [ 1 0];
                case 3, nrm = [0 -1];
                otherwise, nrm = [0 1];
            end
        end
    end
end

for k = 1:size(env.round, 1)
    c = env.round(k, 1:2); r = env.round(k, 3);
    v = p - c; dist = norm(v);
    if dist <= r
        if dist < 1e-12
            if 0 < d, d = 0; nrm = [1 0]; id = 1000 + k; end
        else
            % 内部或表面：d = 0，法向取径向（从圆心穿过 p 的逃逸方向）。
            if 0 < d, d = 0; nrm = v / dist; id = 1000 + k; end
        end
    else
        if dist - r < d
            d = dist - r; nrm = v / dist; id = 1000 + k;
        end
    end
end
end
