function ridge = skel_ridge3d(clear_, free, tol)
%SKEL_RIDGE3D 净空场沿坐标轴的脊线（三维工作空间骨架的近似）。
%
% 判据：若某自由单元在 x、y 或 z 方向上不劣于其两侧邻居（允许容差 tol），
% 则该单元属于脊线。
%
%   clear_ : nx×ny×nz，点到最近障碍物的距离
%   free   : nx×ny×nz logical，自由空间掩码
%   tol    : 容差（通常取 0.5 倍网格尺寸）

c = clear_;
c(~free) = 0;

cxm = c; cxp = c;
cxm(2:end, :, :) = c(1:end - 1, :, :);
cxp(1:end - 1, :, :) = c(2:end, :, :);
rx = c >= max(cxm, cxp) - tol;

cym = c; cyp = c;
cym(:, 2:end, :) = c(:, 1:end - 1, :);
cyp(:, 1:end - 1, :) = c(:, 2:end, :);
ry = c >= max(cym, cyp) - tol;

czm = c; czp = c;
czm(:, :, 2:end) = c(:, :, 1:end - 1);
czp(:, :, 1:end - 1) = c(:, :, 2:end);
rz = c >= max(czm, czp) - tol;

ridge = free & (rx | ry | rz);
end
