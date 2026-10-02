function sk = skel_grid2d(env, opts)
%SKEL_GRID2D 二维工作空间骨架 = 自由空间网格图上的净空加权最短路径（query skeleton）。
%
% 用于 HAS-RRT（Uwacu et al., IEEE RA-L 2025, 10.1109/LRA.2025.3560878）：
% 原文的 DirectAndPruneSkeleton 把工作空间骨架"定向并剪枝"到当前查询，
% 只保留起点→终点相关的骨架路径。本实现以自由空间网格（6 邻域）为骨架图、
% 以净空加权的欧氏长度为边权求最短路：低净空单元的边权被放大，因此路径
% 自动贴近走廊中轴（即中轴的网格近似），再把路径按弧长抽稀为骨架顶点序列。
% 网格尺度不足时自动细化（最多三次），以保证自由空间被正确连通。
%
% 输出 sk 字段：ok, verts (K×2), edgeLen ((K-1)×1), branch (K×1 cell), tBuild。

if nargin < 2, opts = struct; end
def = struct('cell', [], 'maxCells', 45000, 'maxVerts', 400, ...
    'clearWeight', 1.5, 'maxBranch', 2, 'refine', 3);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end

b = env.bounds;
diagLen = hypot(b(2) - b(1), b(4) - b(3));
if isempty(opts.cell)
    cell0 = max(diagLen / 160, sqrt((b(2) - b(1)) * (b(4) - b(3)) / opts.maxCells));
else
    cell0 = opts.cell;
end

sk = struct('ok', false, 'verts', zeros(0, 2), 'edgeLen', zeros(0, 1), ...
    'branch', {cell(0, 1)}, 'tBuild', 0, 'cell', cell0);
t0 = tic;

for attempt = 1:opts.refine
    cs = cell0 / 2^(attempt - 1);
    nx = max(3, floor((b(2) - b(1)) / cs));
    ny = max(3, floor((b(4) - b(3)) / cs));
    xs = b(1) + (0.5:nx) * ((b(2) - b(1)) / nx);
    ys = b(3) + (0.5:ny) * ((b(4) - b(3)) / ny);
    cs = min((b(2) - b(1)) / nx, (b(4) - b(3)) / ny);

    free = false(nx, ny); clear_ = zeros(nx, ny);
    for i = 1:nx
        for j = 1:ny
            p = [xs(i), ys(j)];
            % 自由/占用判定用退化线段（点）的精确碰撞检测：
            % obstacle_query2d 对障碍物内部的点返回"到最近面的距离"，不能用来判内外。
            if collisionChecking(p, p, env.squareAll, env.round)
                free(i, j) = true;
                clear_(i, j) = obstacle_query2d(p, env);
            end
        end
    end
    if nnz(free) < 4, continue; end

    nCell = nx * ny;
    I = []; J = []; W = [];
    cw = opts.clearWeight * cs;
    % 8 邻域：4 个独立方向；边权 = 长度 × (1 + 净空惩罚)，
    % 低净空单元的边权被放大，因此最短路贴近通道中轴。
    offs = [1 0; 0 1; 1 1; 1 -1];
    for oi = 1:size(offs, 1)
        o = offs(oi, :);
        i1 = max(1, 1 - o(1)); i2 = min(nx, nx - o(1));
        j1 = max(1, 1 - o(2)); j2 = min(ny, ny - o(2));
        if i1 > i2 || j1 > j2, continue; end
        A = free(i1:i2, j1:j2);
        Bm = free(i1 + o(1):i2 + o(1), j1 + o(2):j2 + o(2));
        e = A & Bm;
        if ~any(e(:)), continue; end
        [p1, p2] = find(e);
        u = sub2ind([nx ny], p1 + i1 - 1, p2 + j1 - 1);
        v = sub2ind([nx ny], p1 + i1 - 1 + o(1), p2 + j1 - 1 + o(2));
        w = cs * norm(o) * (1 + cw ./ (0.5 * (clear_(u) + clear_(v)) + 0.25 * cs));
        I = [I; u]; J = [J; v]; W = [W; w]; %#ok<AGROW>
    end
    if isempty(I), continue; end

    % --- 接入起点与终点 ---
    freeIdx = find(free);
    for q = 1:2
        if q == 1, p = env.start; else, p = env.goal; end
        pxy = freeIdx2pos(freeIdx, xs, ys, nx, ny);
        d2 = sum((pxy - p).^2, 2);
        [~, ord] = sort(d2);
        nAtt = 0;
        for z = ord(1:min(numel(ord), 12))'
            if collisionChecking(p, pxy(z, :), env.squareAll, env.round)
                I = [I; nCell + q]; J = [J; freeIdx(z)]; %#ok<AGROW>
                W = [W; norm(pxy(z, :) - p)]; %#ok<AGROW>
                nAtt = nAtt + 1;
                if nAtt >= 4, break; end
            end
        end
    end

    G = graph(I, J, W, nCell + 2);
    [seq, dLen] = shortestpath(G, nCell + 1, nCell + 2);
    if isempty(seq) || ~isfinite(dLen), continue; end

    % --- 顶点序列 ---
    pos = zeros(numel(seq), 2);
    for z = 1:numel(seq)
        if seq(z) == nCell + 1, pos(z, :) = env.start;
        elseif seq(z) == nCell + 2, pos(z, :) = env.goal;
        else
            [ii, jj] = ind2sub([nx ny], seq(z));
            pos(z, :) = [xs(ii), ys(jj)];
        end
    end
    % 按弧长抽稀（相邻顶点间距不小于 cs），不做视线捷径化
    if size(pos, 1) > 2 * opts.maxVerts + 2
        keep = false(size(pos, 1), 1);
        keep(1) = true; keep(end) = true;
        step = ceil(size(pos, 1) / opts.maxVerts);
        keep(2:step:end - 1) = true;
        pos = pos(keep, :);
    end
    simp = pos(1, :);
    for z = 2:size(pos, 1) - 1
        if norm(pos(z, :) - simp(end, :)) >= cs
            simp(end + 1, :) = pos(z, :); %#ok<AGROW>
        end
    end
    simp(end + 1, :) = pos(end, :);
    if size(simp, 1) < 2, continue; end
    simp(1, :) = env.start; simp(end, :) = env.goal;

    % --- 顶点处的分支区域（转向处的自由邻居，不在路径上）---
    K = size(simp, 1);
    branch = cell(K, 1);
    onPath = false(nCell, 1);
    for z = 1:K
        d2 = sum((pxy - simp(z, :)).^2, 2);
        onPath(freeIdx(d2 <= 4 * cs^2)) = true;
    end
    for z = 2:K - 1
        dPrev = simp(z, :) - simp(z - 1, :); dPrev = dPrev / max(norm(dPrev), eps);
        if z + 1 <= K
            dNext = simp(z + 1, :) - simp(z, :); dNext = dNext / max(norm(dNext), eps);
            if dot(dPrev, dNext) > 0.9, continue; end        % 直行处不设分支
        end
        [ii, jj] = nearest_cell(simp(z, :), xs, ys);
        cand = zeros(0, 2);
        for di = [-1 0 1]
            for dj = [-1 0 1]
                if di == 0 && dj == 0, continue; end
                i2 = ii + di; j2 = jj + dj;
                if i2 < 1 || i2 > nx || j2 < 1 || j2 > ny, continue; end
                if ~free(i2, j2), continue; end
                p2 = [xs(i2), ys(j2)];
                if collisionChecking(simp(z, :), p2, env.squareAll, env.round)
                    cand(end + 1, :) = p2; %#ok<AGROW>
                end
            end
        end
        if ~isempty(cand)
            cand = cand(randperm(size(cand, 1), min(size(cand, 1), opts.maxBranch)), :);
            branch{z} = cand;
        end
    end

    sk.ok = true;
    sk.verts = simp;
    sk.edgeLen = sqrt(sum(diff(simp).^2, 2));
    sk.branch = branch;
    sk.cell = cs;
    sk.tBuild = toc(t0);
    return;
end

sk.tBuild = toc(t0);
end

function pxy = freeIdx2pos(freeIdx, xs, ys, nx, ny)
[ii, jj] = ind2sub([nx ny], freeIdx);
pxy = [xs(ii(:))', ys(jj(:))'];
end

function [ii, jj] = nearest_cell(p, xs, ys)
[~, ii] = min(abs(xs - p(1)));
[~, jj] = min(abs(ys - p(2)));
end
