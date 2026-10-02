function ridge = skel_ridge2d(clear_, free, tol)
%SKEL_RIDGE2D 净空场沿坐标轴的脊线（二维工作空间骨架的近似）。
%
% 判据：若某自由单元在 x 或 y 方向上不劣于其两侧邻居（允许容差 tol），
% 则该单元属于脊线。相比"3×3 邻域局部极大"判据，该定义保留走廊中线上
% 的整条线段，得到的骨架网络连通性更好（中轴与自由空间同为连通集）。
%
%   clear_ : nx×ny，点到最近障碍物的距离（障碍物内部单元可为任意值）
%   free   : nx×ny logical，自由空间掩码
%   tol    : 容差（通常取 0.5 倍网格尺寸）

c = clear_;
c(~free) = 0;

cxm = c; cxp = c;
cxm(2:end, :) = c(1:end - 1, :);
cxp(1:end - 1, :) = c(2:end, :);
rx = c >= max(cxm, cxp) - tol;

cym = c; cyp = c;
cym(:, 2:end) = c(:, 1:end - 1);
cyp(:, 1:end - 1) = c(:, 2:end);
ry = c >= max(cym, cyp) - tol;

ridge = free & (rx | ry);
end
