function [path, info] = bitstar_nd(bounds, start, goal, isFree, opts)
%BITSTAR_ND  BIT*（批处理知情采样树）维度通用实现（2D/3D 共用）。
%
%   [path, info] = bitstar_nd(bounds, start, goal, isFree, opts)
%
% 实现要点（对应 Gammell et al., 2015 的批处理版本）：
%   1) 采样集 S 按批次增长；找到首个解后改为知情椭球采样；
%   2) 树顶点按 f = g + h 的最佳优先顺序扩展（每批重排队列）；
%   3) 新样本仅通过一次最佳父节点搜索接入树（边采用即时求值）；
%   4) 扩展时对邻域树顶点尝试重布线；
%   5) 每批结束后尝试把目标点接入树，得到 anytime 上界更新。
%
% opts 字段（默认值）：
%   maxNodes (2000)  采样点预算（不含起终点）
%   batchSize (200)  每批采样数
%   gamma    (0.9)   连接半径系数 r(n)=gamma*(log n/n)^(1/d)*scale
%
% 输出 info 与其它规划器一致（含 firstSolutionTime / bestHistory / 计时拆分）。

if nargin < 5, opts = struct; end
def = struct('maxNodes', 2000, 'batchSize', 200, 'gamma', 0.9);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
start = double(start(:)'); goal = double(goal(:)');
d = numel(start);
lo = bounds(1:2:end); hi = bounds(2:2:end);
ext = hi - lo; scale = prod(ext)^(1/d);

maxN = max(10, round(opts.maxNodes));
batchSize = max(10, round(opts.batchSize));
batches = max(1, ceil(maxN / batchSize));

% 树结构
V = zeros(maxN + 4, d); C = inf(maxN + 4, 1); Par = zeros(maxN + 4, 1);
V(1, :) = start; C(1) = 0; nV = 1;
% 采样池
S = zeros(maxN + 2, d); alive = false(maxN + 2, 1); nS = 0;

best = inf; goalFrom = 0; firstSolTime = NaN;
hist = zeros(4096, 2); nh = 0;
t0 = tic;
nSampled = 0;

if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end
tWall = tic;                      % 等时间对照用：墙钟上限（默认 Inf 表示不启用）
for b = 1:batches
    if toc(tWall) > opts.timeLimit, break; end
    k = min(batchSize, maxN - nSampled);
    if k <= 0, break; end
    if isfinite(best)
        newS = sample_informed(lo, hi, start, goal, best, k);
    else
        newS = lo + ext .* rand(k, d);
    end
    S(nS + (1:k), :) = newS; alive(nS + (1:k)) = true;
    nS = nS + k; nSampled = nSampled + k;

    nAll = nV + sum(alive(1:nS));
    r = gamma_n(nAll, d, scale, opts.gamma);
    % 空间哈希网格：树顶点网格在接入新顶点时增量更新；样本网格在批内保持不变
    [binsV, gdims] = grid_hash_build(V(1:nV, :), lo, r, []);
    aS = find(alive(1:nS));
    if isempty(aS)
        binsS = cell(1, prod(gdims)); sdims = gdims;
    else
        [binsS, sdims] = grid_hash_build(S(aS, :), lo, r, []);
    end

    % ---- 最佳优先扩展（每批从所有树顶点重排队列）----
    expanded = false(nV, 1);
    nPop = 0;
    while true
    if toc(tWall) > opts.timeLimit, break; end
        hV = vecnorm(V(1:nV, :) - goal, 2, 2);
        fV = C(1:nV) + hV;
        fV(expanded) = inf;
        [fmin, vi] = min(fV);
        if ~isfinite(fmin) || fmin >= best, break; end
        nPop = nPop + 1;
        if nPop > 4 * nV + 500, break; end      % 迭代上限，保证有界运行时间
        expanded(vi) = true;
        v = V(vi, :);

        % (a) 邻域样本接入树（空间哈希取候选）
        if ~isempty(aS)
            sIdx = grid_hash_query(binsS, sdims, lo, r, v);
            cand = aS(sIdx);
            cand = cand(alive(cand));
            if ~isempty(cand)
                dc = vecnorm(S(cand, :) - v, 2, 2);
                keep = dc <= r;
                cand = cand(keep);
                [~, o] = sort(dc(keep));
                cand = cand(o);
                for zz = 1:numel(cand)
                    s = cand(zz);
                    if ~alive(s), continue; end
                    xs = S(s, :);
                    % 在树顶点邻域内搜索最优父节点（按代价排序后逐条碰撞检测）
                    [vIdx, linV] = grid_hash_query(binsV, gdims, lo, r, xs);
                    vIdx = vIdx(vIdx <= nV);
                    if isempty(vIdx), continue; end
                    dVv = vecnorm(V(vIdx, :) - xs, 2, 2);
                    keep2 = dVv <= r;
                    pool = vIdx(keep2); dP = dVv(keep2);
                    if isempty(pool), continue; end
                    [~, o2] = sort(C(pool) + dP);
                    pool = pool(o2); dP = dP(o2);
                    for yy = 1:numel(pool)
                        y = pool(yy);
                        if logical(isFree(V(y, :), xs))
                            nV = nV + 1;
                            V(nV, :) = xs; C(nV) = C(y) + dP(yy); Par(nV) = y;
                            expanded(nV, 1) = false;
                            binsV{linV} = [binsV{linV}, nV];
                            alive(s) = false;
                            break;
                        end
                    end
                end
            end
        end

        % (b) 重布线（空间哈希取候选）
        vNb = grid_hash_query(binsV, gdims, lo, r, v);
        vNb = vNb(vNb <= nV);
        dNb = vecnorm(V(vNb, :) - v, 2, 2);
        okNb = dNb <= r;
        nb = vNb(okNb); dNb = dNb(okNb);
        for zz = 1:numel(nb)
            u = nb(zz);
            if u == vi, continue; end
            nc = C(vi) + dNb(zz);
            if nc < C(u) - 1e-12 && logical(isFree(v, V(u, :)))
                C(u) = nc; Par(u) = vi;
                expanded(u) = false;      % 允许下批/后续再扩展
            end
        end
    end

    % ---- 目标接入尝试（anytime 上界）----
    dG = vecnorm(V(1:nV, :) - goal, 2, 2);
    [~, og] = sort(C(1:nV) + dG);
    for zz = 1:min(50, nV)
        y = og(zz);
        gc = C(y) + dG(y);
        if gc < best && logical(isFree(V(y, :), goal))
            best = gc; goalFrom = y;
            if isnan(firstSolTime), firstSolTime = toc(t0); end
            nh = nh + 1; hist(nh, :) = [toc(t0), best];
            break;
        end
    end

    % ---- 采样池压缩 ----
    if any(~alive(1:nS))
        keep = find(alive(1:nS));
        S(1:numel(keep), :) = S(keep, :);
        alive = false(size(alive)); alive(1:numel(keep)) = true;
        nS = numel(keep);
    end
end

tPlanner = toc(t0);
if goalFrom > 0
    q = goalFrom; way = goal;
    while q > 0
        way(end + 1, :) = V(q, :); %#ok<AGROW>
        q = Par(q);
    end
    way = flipud(way);
    path = way; tShortcut = 0;   % 终路径贪心捷径后处理已取消（2026-09-20）
    L = length_nd(path);
else
    path = zeros(0, d); tShortcut = 0; L = inf;
end

hist = hist(1:nh, :);
info = struct('success', goalFrom > 0, 'length', L, 'nodes', nV, 'iterations', nSampled, ...
    'firstSolutionTime', firstSolTime, 'bestHistory', hist, 'samples', V(1:nV, :), ...
    'tPlanner', tPlanner, 'tShortcut', tShortcut, 'timeTotal', toc(t0));
end

function r = gamma_n(n, d, scale, gamma)
r = gamma * (log(n) / n)^(1 / d) * scale;
end

function Xs = sample_informed(lo, hi, s, g, c, k)
% 椭球知情采样：单位球均匀采样后缩放旋转平移（与 Informed-RRT* 一致）。
d = numel(s);
X0 = randn(k, d);
normX = vecnorm(X0, 2, 2);
X0 = X0 ./ max(normX, eps);                 % 单位球面
u = rand(k, 1).^(1 / d);                    % 径向缩放（单位球内均匀）
X0 = X0 .* u;
cbest = c; dg = norm(g - s);
if cbest <= dg
    Xs = lo + (hi - lo) .* rand(k, d); return;
end
a = cbest / 2; b = sqrt(max(cbest^2 - dg^2, eps)) / 2;
e = (g - s) / dg;
ref = zeros(1, d); ref(end) = 1;
if abs(dot(e, ref)) > 0.9, ref = zeros(1, d); ref(1) = 1; end
if d == 2
    v = [-e(2), e(1)];
    rot = [e' v'];
else
    v = cross(e, ref); v = v / norm(v);
    w = cross(e, v);
    rot = [e' v' w'];
end
Xs = X0 * (diag([a, repmat(b, 1, d - 1)])) * rot' + (s + g) / 2;
end

function p = shortcut_nd(p, isFree)
if size(p, 1) < 3, return; end
i = 1;
while i < size(p, 1) - 1
    j = size(p, 1);
    while j > i + 1 && ~logical(isFree(p(i, :), p(j, :))), j = j - 1; end
    p = [p(1:i, :); p(j:end, :)]; %#ok<AGROW>
    i = i + 1;
end
end

function L = length_nd(p)
if size(p, 1) < 2, L = inf; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
