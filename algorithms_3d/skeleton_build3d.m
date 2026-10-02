function sk = skeleton_build3d(env, opts)
%SKELETON_BUILD3D 三维工作空间骨架的构建、定向与剪枝（HAS-RRT 预规划阶段）。
%
% 对应原文的 DirectAndPruneSkeleton：把工作空间骨架定向、剪枝到当前查询，
% 只保留起点->终点相关的骨架路径，作为后续采样区域的锚定对象。
% 具体实现见 skel_grid3d（自由空间网格图上的净空加权最短路径）。
%
% 输出 sk 字段：ok, verts (K×3), edgeLen ((K-1)×1), branch (K×1 cell), tBuild。

if nargin < 2, opts = struct; end
sk = skel_grid3d(env, opts);
end
