function [path, info] = fmtstar_nd(bounds, start, goal, isFree, opts)
%FMTSTAR_ND  FMT*（快速行进树）维度通用实现（2D/3D 共用）。
%
%   [path, info] = fmtstar_nd(bounds, start, goal, isFree, opts)
%
% opts 字段（默认值）：
%   maxNodes (2000)   总采样节点预算 N_max
%   batchSize (200)   每批采样数
%   gamma    (1.0)    连接半径系数 r(n)=gamma*(log n/n)^(1/d)*scale
%   goalTol  (1e-6)   判定到达目标点的距离容差（点采样直接命中）
%
% 输出 info：success, length, nodes, iterations, firstSolutionTime,
% bestHistory, tPlanner, tShortcut, timeTotal。
%
% 实现要点（与 FMT* 一致的核心流程）：
%   1) 分批采样，节点集 X 随批次增长；
%   2) 维护开集 X_open（树内可扩展节点），按 g(z)+h(z) 升序扩展；
%   3) 对 z 的邻域未访问节点 x，在已入树节点（g 有限）中选代价最小的
%      “预测父节点”y，仅对边 (y,x) 做碰撞检测（惰性思想）；
%   4) 达到目标点后继续运行剩余批次，路径代价可继续改善（anytime）。

if nargin < 5, opts = struct; end
def = struct('maxNodes', 2000, 'batchSize', 200, 'gamma', 1.0, 'goalTol', 1e-6);
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
batches = max(1, ceil((maxN - 2) / batchSize));

X = zeros(maxN + 2, d); g = inf(maxN + 2, 1); P = zeros(maxN + 2, 1);
inOpen = false(maxN + 2, 1); inTree = false(maxN + 2, 1);
X(1, :) = start; g(1) = 0; inOpen(1) = true; inTree(1) = true;
X(2, :) = goal;                     % 目标点作为固定样本参与接入
n = 2;
goalIdx = 0; best = inf;
hist = zeros(4096, 2); nh = 0;
nPop = 0;
firstSolTime = NaN;
t0 = tic;

if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end
tWall = tic;                      % 等时间对照用：墙钟上限（默认 Inf 表示不启用）
for b = 1:batches
    if toc(tWall) > opts.timeLimit, break; end
    k = min(batchSize, (maxN + 2) - n);
    if k <= 0, break; end
    X(n + (1:k), :) = lo + ext .* rand(k, d);
    n = n + k;
    r = gamma_n(n, d, scale, opts.gamma);
    % 每批开始时重新开放全部树节点，使新样本可与既有树连接
    % （对应原 FMT* 伪码中 V_open <- V_open_old 的做法）。
    nPop = 0;
    inOpen(1:n) = isfinite(g(1:n));
    % 空间哈希：把邻域查询从 O(n) 降为 O(局部候选)
    [binsX, gdims] = grid_hash_build(X(1:n, :), lo, r, []);

    while true
    if toc(tWall) > opts.timeLimit, break; end
        % 从开集中取 f = g + h 最小的节点（只在开集内计算，避免全量扫描）
        openIdx = find(inOpen(1:n));
        if isempty(openIdx), break; end
        nPop = nPop + 1;
        if nPop > 4 * n + 500, break; end      % 迭代上限，保证有界运行时间
        fv = g(openIdx) + vecnorm(X(openIdx, :) - goal, 2, 2);
        [fmin, ii] = min(fv);
        if ~isfinite(fmin) || fmin >= best, break; end   % 开集全被剪枝
        z = openIdx(ii);
        inOpen(z) = false;
        % z 的邻域内未访问节点（不在树中的）—— 空间哈希取候选
        nbz = grid_hash_query(binsX, gdims, lo, r, X(z, :));
        nbz = nbz(nbz ~= z);
        dz = sqrt(sum((X(nbz, :) - X(z, :)).^2, 2));
        cand = nbz(dz <= r & ~inTree(nbz));
        if ~isempty(cand)
            [~, ord] = sort(sqrt(sum((X(cand, :) - X(z, :)).^2, 2)));
            cand = cand(ord);
            for zz = 1:numel(cand)
                x = cand(zz);
                % 在已入树节点中寻找最优父节点（先按代价排序，依次做碰撞检测）
                nbx = grid_hash_query(binsX, gdims, lo, r, X(x, :));
                nbx = nbx(nbx ~= x);
                dx = sqrt(sum((X(nbx, :) - X(x, :)).^2, 2));
                pool = nbx(isfinite(g(nbx)) & dx <= r);
                if isempty(pool), continue; end
                dP = sqrt(sum((X(pool, :) - X(x, :)).^2, 2));
                [~, o2] = sort(g(pool) + dP);
                pool = pool(o2);
                dP = dP(o2);
                for yy = 1:numel(pool)
                    y = pool(yy);
                    if logical(isFree(X(y, :), X(x, :)))
                        g(x) = g(y) + dP(yy);
                        P(x) = y;
                        inTree(x) = true; inOpen(x) = true;
                        break;
                    end
                end
                % 目标点是否已加入树（点采样直接落在目标上）
                if x == 2
                    if isfinite(g(2)) && g(2) < best
                        best = g(2); goalIdx = 2;
                        if isnan(firstSolTime), firstSolTime = toc(t0); end
                        nh = nh + 1; hist(nh, :) = [toc(t0), best];
                    end
                end
            end
        end
    end

    % 目标点显式接入/改进尝试：避免“目标附近恰好无样本”导致的伪失败。
    dG = sqrt(sum((X(1:n, :) - X(2, :)).^2, 2));
    treeIdx = find(isfinite(g(1:n)));
    treeIdx(treeIdx == 2) = [];
    if ~isempty(treeIdx)
        [~, ordG] = sort(g(treeIdx) + dG(treeIdx));
        for zz = 1:min(50, numel(ordG))
            y = treeIdx(ordG(zz));
            gc = g(y) + dG(y);
            if gc < best - 1e-12 && logical(isFree(X(y, :), X(2, :)))
                g(2) = gc; P(2) = y; inTree(2) = true; inOpen(2) = true;
                best = gc; goalIdx = 2;
                if isnan(firstSolTime), firstSolTime = toc(t0); end
                nh = nh + 1; hist(nh, :) = [toc(t0), best];
                break;
            end
        end
    end
end

tPlanner = toc(t0);
if goalIdx > 0    q = goalIdx; way = X(q, :);
    while P(q) ~= 0, q = P(q); way(end + 1, :) = X(q, :); end %#ok<AGROW>
    way = flipud(way);
    path = way; tShortcut = 0;   % 终路径贪心捷径后处理已取消（2026-09-20）
    L = length_nd(path);
else
    path = zeros(0, d); tShortcut = 0; L = inf;
end

hist = hist(1:nh, :);
info = struct('success', goalIdx > 0, 'length', L, 'nodes', n, 'iterations', n - 2, ...
    'firstSolutionTime', firstSolTime, 'bestHistory', hist, 'samples', X(1:n, :), ...
    'tPlanner', tPlanner, 'tShortcut', tShortcut, 'timeTotal', toc(t0));
end

function r = gamma_n(n, d, scale, gamma)
r = gamma * (log(n) / n)^(1 / d) * scale;
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
