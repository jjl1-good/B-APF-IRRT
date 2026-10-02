function sk = skeleton_build2d(env, opts)
%SKELETON_BUILD2D 二维工作空间骨架的构建、定向与剪枝（HAS-RRT 预规划阶段）。
%
% 对应原文的 DirectAndPruneSkeleton：把工作空间骨架定向、剪枝到当前查询，
% 只保留起点->终点相关的骨架路径，作为后续采样区域的锚定对象。
% 具体实现见 skel_grid2d（自由空间网格图上的净空加权最短路径）。
%
% 输出 sk 字段：
%   ok      : 是否成功得到起点->终点的骨架路径
%   verts   : K×2，有序骨架顶点（verts(1,:)=起点，verts(K,:)=终点）
%   edgeLen : (K-1)×1，各段长度
%   branch  : K×1 cell，每个顶点的分支邻居位置（M×2）
%   tBuild  : 构建耗时（秒）

if nargin < 2, opts = struct; end
sk = skel_grid2d(env, opts);
end
