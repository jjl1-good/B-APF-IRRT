function [bins, dims] = grid_hash_build(pts, lo, cs, dims)
%GRID_HASH_BUILD 构建空间哈希网格，用于批量规划器的邻域查询。
%
%   [bins, dims] = grid_hash_build(pts, lo, cs)
%   [bins, dims] = grid_hash_build(pts, lo, cs, dims)
%
% pts  : N×d 点集（d = 2 或 3）
% lo   : 1×d 网格原点（通常取环境下界）
% cs   : 网格边长（应 ≥ 邻域半径 r）
% dims : 可选的网格维度（保持多次构建一致）
% bins : 1×prod(dims) 的 cell 数组，bins{k} 保存落在该格中的点索引
%
% 与 grid_hash_query 配合使用，可把 FMT*/BIT* 等算法中的 O(n) 全量
% 邻域扫描降为 O(局部点数)，使三维大规模实验可行。

d = size(pts, 2);
if nargin < 4 || isempty(dims)
    dims = max(1, ceil((max(pts, [], 1) - lo) ./ cs));
end
dims = dims(:)';
bins = cell(1, prod(dims));

key = floor((pts - lo) ./ cs);
key = min(max(key, 0), repmat(dims, size(key, 1), 1) - 1);
lin = key(:, 1) + 1;
if d >= 2
    lin = lin + key(:, 2) * dims(1);
end
if d >= 3
    lin = lin + key(:, 3) * dims(1) * dims(2);
end
for i = 1:size(pts, 1)
    bins{lin(i)} = [bins{lin(i)}, i];
end
end
