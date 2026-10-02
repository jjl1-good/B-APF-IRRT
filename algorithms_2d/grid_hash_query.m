function [idx, lin] = grid_hash_query(bins, dims, lo, cs, p)
%GRID_HASH_QUERY 查询点 p 周围 3×3（三维为 3×3×3）网格内的点索引。
%
%   idx = grid_hash_query(bins, dims, lo, cs, p)
%   [idx, lin] = grid_hash_query(...)
%
% 返回候选点索引（未做精确距离过滤，由调用方按半径 r ≤ cs 过滤）。
% lin 为该点自身所在格的线性索引，便于把新点追加进网格。

d = numel(p);
dims = dims(:)';
key = floor((p - lo) ./ cs);
key = min(max(key, 0), dims - 1);
lin = key(1) + 1;
if d >= 2
    lin = lin + key(2) * dims(1);
end
if d >= 3
    lin = lin + key(3) * dims(1) * dims(2);
end

idx = [];
for ox = -1:1
    for oy = -1:1
        if d < 2 && oy ~= 0, continue; end
        for oz = -1:1
            if d < 3 && oz ~= 0, continue; end
            kk = key;
            kk(1) = kk(1) + ox;
            if d >= 2, kk(2) = kk(2) + oy; end
            if d >= 3, kk(3) = kk(3) + oz; end
            if any(kk < 0) || any(kk >= dims), continue; end
            l = kk(1) + 1;
            if d >= 2, l = l + kk(2) * dims(1); end
            if d >= 3, l = l + kk(3) * dims(1) * dims(2); end
            if ~isempty(bins{l})
                idx = [idx, bins{l}]; %#ok<AGROW>
            end
        end
    end
end
idx = idx(:);      % 统一为列向量，便于与向量化的距离列向量做逻辑索引
end
