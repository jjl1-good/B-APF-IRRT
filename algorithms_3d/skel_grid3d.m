function sk = skel_grid3d(env, opts)
%SKEL_GRID3D 三维工作空间骨架 = 自由空间网格图上的净空加权最短路径。
%
% 与 skel_grid2d 对应（HAS-RRT 的 DirectAndPruneSkeleton）：
%   1) 在包围盒上建立均匀网格，用退化线段（点）的精确碰撞检测判定自由单元；
%   2) 以 6 邻域建立自由空间网格图，边权 = 长度 × (1 + 净空惩罚)，
%      低净空单元的边权被放大，路径因此贴近通道中轴；
%   3) 起点与终点接入最近的若干自由单元（线段碰撞检测），求最短路径；
%   4) 网格过粗导致不连通时自动细化（最多 refine 次）。
%
% 输出 sk 字段：ok, verts (K×3), edgeLen ((K-1)×1), branch (K×1 cell), tBuild, cell。

if nargin < 2, opts = struct; end
def = struct('cell', [], 'maxCells', 45000, 'maxVerts', 400, ...
    'clearWeight', 1.5, 'maxBranch', 2, 'refine', 3);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end

b = env.bounds;
vol = (b(2) - b(1)) * (b(4) - b(3)) * (b(6) - b(5));
diagLen = norm([b(2) - b(1), b(4) - b(3), b(6) - b(5)]);
if isempty(opts.cell)
    cell0 = max(diagLen / 40, (vol / opts.maxCells)^(1 / 3));
else
    cell0 = opts.cell;
end

sk = struct('ok', false, 'verts', zeros(0, 3), 'edgeLen', zeros(0, 1), ...
    'branch', {cell(0, 1)}, 'tBuild', 0, 'cell', cell0);
t0 = tic;

for attempt = 1:opts.refine
    cs = cell0 / 2^(attempt - 1);
    nx = max(3, floor((b(2) - b(1)) / cs));
    ny = max(3, floor((b(4) - b(3)) / cs));
    nz = max(3, floor((b(6) - b(5)) / cs));
    xs = b(1) + (0.5:nx) * ((b(2) - b(1)) / nx);
    ys = b(3) + (0.5:ny) * ((b(4) - b(3)) / ny);
    zs = b(5) + (0.5:nz) * ((b(6) - b(5)) / nz);
    cs = min([(b(2) - b(1)) / nx, (b(4) - b(3)) / ny, (b(6) - b(5)) / nz]);

    free = false(nx, ny, nz); clear_ = zeros(nx, ny, nz);
    for i = 1:nx
        for j = 1:ny
            for k = 1:nz
                p = [xs(i), ys(j), zs(k)];
                if collisionChecking3D(p, p, env)
                    free(i, j, k) = true;
                    clear_(i, j, k) = obstacle_query3d(p, env);
                end
            end
        end
    end
    if nnz(free) < 4, continue; end

    nCell = nx * ny * nz;
    I = []; J = []; W = [];
    cw = opts.clearWeight * cs;
    dims = [nx ny nz];
    % 26 邻域：13 个独立方向；边权 = 长度 × (1 + 净空惩罚)。
    offs = [1 0 0; 0 1 0; 0 0 1; ...
        1 1 0; 1 -1 0; 1 0 1; 1 0 -1; 0 1 1; 0 1 -1; ...
        1 1 1; 1 1 -1; 1 -1 1; 1 -1 -1];
    for oi = 1:size(offs, 1)
        o = offs(oi, :);
        i1 = max(1, 1 - o(1)); i2 = min(nx, nx - o(1));
        j1 = max(1, 1 - o(2)); j2 = min(ny, ny - o(2));
        k1 = max(1, 1 - o(3)); k2 = min(nz, nz - o(3));
        if i1 > i2 || j1 > j2 || k1 > k2, continue; end
        A = free(i1:i2, j1:j2, k1:k2);
        Bm = free(i1 + o(1):i2 + o(1), j1 + o(2):j2 + o(2), k1 + o(3):k2 + o(3));
        e = A & Bm;
        if ~any(e(:)), continue; end
        [p1, p2, p3] = ind2sub(size(A), find(e));
        u = sub2ind(dims, p1 + i1 - 1, p2 + j1 - 1, p3 + k1 - 1);
        v = sub2ind(dims, p1 + i1 - 1 + o(1), p2 + j1 - 1 + o(2), p3 + k1 - 1 + o(3));
        w = cs * norm(o) * (1 + cw ./ (0.5 * (clear_(u) + clear_(v)) + 0.25 * cs));
        I = [I; u]; J = [J; v]; W = [W; w]; %#ok<AGROW>
    end
    if isempty(I), continue; end

    freeIdx = find(free);
    [fi, fj, fk] = ind2sub(dims, freeIdx);
    pxy = [xs(fi(:))', ys(fj(:))', zs(fk(:))'];
    for q = 1:2
        if q == 1, p = env.start; else, p = env.goal; end
        d2 = sum((pxy - p).^2, 2);
        [~, ord] = sort(d2);
        nAtt = 0;
        for z = ord(1:min(numel(ord), 16))'
            if collisionChecking3D(p, pxy(z, :), env)
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

    pos = zeros(numel(seq), 3);
    for z = 1:numel(seq)
        if seq(z) == nCell + 1, pos(z, :) = env.start;
        elseif seq(z) == nCell + 2, pos(z, :) = env.goal;
        else
            [ii, jj, kk] = ind2sub(dims, seq(z));
            pos(z, :) = [xs(ii), ys(jj), zs(kk)];
        end
    end
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

    % --- 转向处的分支区域 ---
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
            if dot(dPrev, dNext) > 0.9, continue; end
        end
        ii = nearest_idx(xs, simp(z, 1));
        jj = nearest_idx(ys, simp(z, 2));
        kk = nearest_idx(zs, simp(z, 3));
        cand = zeros(0, 3);
        for di = [-1 0 1]
            for dj = [-1 0 1]
                for dk = [-1 0 1]
                    if di == 0 && dj == 0 && dk == 0, continue; end
                    i2 = ii + di; j2 = jj + dj; k2 = kk + dk;
                    if i2 < 1 || i2 > nx || j2 < 1 || j2 > ny || k2 < 1 || k2 > nz, continue; end
                    if ~free(i2, j2, k2), continue; end
                    p2 = [xs(i2), ys(j2), zs(k2)];
                    if collisionChecking3D(simp(z, :), p2, env)
                        cand(end + 1, :) = p2; %#ok<AGROW>
                    end
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

function i = nearest_idx(v, x)
[~, i] = min(abs(v - x));
end
