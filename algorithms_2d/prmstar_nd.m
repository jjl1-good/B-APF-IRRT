function [path, info] = prmstar_nd(bounds, start, goal, isFree, opts)
%PRMSTAR_ND  PRM*（渐近最优概率路线图）维度通用实现（2D/3D 共用）。
%
%   [path, info] = prmstar_nd(bounds, start, goal, isFree, opts)
%
% bounds : 1×2d 边界 [xmin xmax ymin ymax (zmin zmax)]。
% start/goal : 1×d 起点与终点。
% isFree : 函数句柄 isFree(a,b)，线段 (a,b) 无碰撞时返回 true。
% opts 字段（默认值）：
%   maxNodes (2000)   采样节点预算（对应论文 N_max）
%   gamma    (1.2)    连接半径系数 r(n)=gamma*(log n/n)^(1/d)*scale
%   kMax     (15)     每个新节点的最大连接尝试次数（含最近邻保底）
%   goalBias (0.0)    对目标点的额外采样概率（保持 PRM* 原始形式为 0）
%
% 输出 info：
%   success, length, nodes, iterations
%   firstSolutionTime  首次连通起终点（并求得路径）的时间
%   bestHistory        M×2 [t, cost]，连通/优化更新记录
%   tPlanner, tShortcut, timeTotal
%
% 说明：PRM* 为多查询图方法，此处按单次规划任务运行，构建完成后在图
% 上做 Dijkstra 查询；首次连通时执行一次查询用于 anytime 对比。

if nargin < 5, opts = struct; end
def = struct('maxNodes', 2000, 'gamma', 1.2, 'kMax', 15, 'goalBias', 0.0);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
start = double(start(:)'); goal = double(goal(:)');
d = numel(start);
lo = bounds(1:2:end); hi = bounds(2:2:end);
ext = hi - lo; scale = prod(ext)^(1/d);

maxN = max(10, round(opts.maxNodes));
V = zeros(maxN + 2, d); P = zeros(maxN + 2, 1); G = inf(maxN + 2, 1);
V(1, :) = start; G(1) = 0;
V(2, :) = goal;
n = 2;
EI = zeros(maxN * opts.kMax, 1); EJ = zeros(maxN * opts.kMax, 1);
nE = 0;

% 并查集用于首次连通检测
uf = 1:(maxN + 2);
connected = false;
firstSolTime = NaN; best = inf;
hist = zeros(256, 2); nh = 0;
t0 = tic;

if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end
tWall = tic;                      % 等时间对照用：墙钟上限（默认 Inf 表示不启用）
for it = 1:maxN
    if toc(tWall) > opts.timeLimit, break; end
    if opts.goalBias > 0 && rand < opts.goalBias
        x = goal;
    else
        x = lo + ext .* rand(1, d);
    end
    n = n + 1;
    V(n, :) = x;
    r = radius_n(n, d, scale, opts.gamma);

    dist = sqrt(sum((V(1:n-1, :) - x).^2, 2));
    [ds, ord] = sort(dist);
    nTry = min(opts.kMax, numel(ds));
    for z = 1:nTry
        j = ord(z);
        if ds(z) <= r || z == 1
            if logical(isFree(V(j, :), x))
                nE = nE + 1; EI(nE) = j; EJ(nE) = n;
                uf = uf_union(uf, j, n);
            end
        end
    end

    if ~connected && uf_find(uf, 1) == uf_find(uf, 2)
        connected = true;
        [cost, waypts] = graph_query(V(1:n, :), EI(1:nE), EJ(1:nE), 1, 2);
        if isfinite(cost)
            firstSolTime = toc(t0);
            best = cost;
            nh = nh + 1; hist(nh, :) = [firstSolTime, cost];
            if nh == 1, lastWay = waypts; end
        end
    end
end

tPlanner = toc(t0);
[cost, waypts] = graph_query(V(1:n, :), EI(1:nE), EJ(1:nE), 1, 2);
if isfinite(cost)
    if cost < best || ~connected
        best = cost; nh = nh + 1; hist(nh, :) = [toc(t0), cost];
    end
    lastWay = waypts;
end
tPlanner = toc(t0);

if isfinite(best)
    path = lastWay; tShortcut = 0;   % 终路径贪心捷径后处理已取消（2026-09-20）
    L = length_nd(path);
else
    path = zeros(0, d); tShortcut = 0; L = inf;
end

hist = hist(1:nh, :);
info = struct('success', isfinite(best), 'length', L, 'nodes', n, ...
    'iterations', n - 2, 'firstSolutionTime', firstSolTime, ...
    'bestHistory', hist, 'tPlanner', tPlanner, 'tShortcut', tShortcut, ...
    'samples', V(1:n, :), 'timeTotal', toc(t0));
end

function r = radius_n(n, d, scale, gamma)
r = gamma * (log(n) / n)^(1 / d) * scale;
end

function [cost, way] = graph_query(V, EI, EJ, si, gi)
% 稀疏图上的 Dijkstra（O(n^2) 实现，n 为节点数，节点规模 ≤ 数千）。
n = size(V, 1);
dist = inf(n, 1); prev = zeros(n, 1); done = false(n, 1);
dist(si) = 0;
segW = sqrt(sum((V(EI, :) - V(EJ, :)).^2, 2));
E = numel(EI);
adjI = [EI; EJ]; adjJ = [EJ; EI]; adjW = [segW; segW];
[~, ord] = sort(adjI); adjI = adjI(ord); adjJ = adjJ(ord); adjW = adjW(ord);
head = zeros(n + 1, 1);
for k = 1:E * 2
    head(adjI(k) + 1) = head(adjI(k) + 1) + 1;
end
head = cumsum(head);
pos = head;
for it = 1:n
    u = -1; du = inf;
    for k = 1:n
        if ~done(k) && dist(k) < du, du = dist(k); u = k; end
    end
    if u < 0, break; end
    done(u) = true;
    if u == gi, break; end
    for k = (pos(u) + 1):head(u + 1)
        v = adjJ(k); nd = du + adjW(k);
        if nd < dist(v), dist(v) = nd; prev(v) = u; end
    end
end
if isfinite(dist(gi))
    cost = dist(gi);
    q = gi; way = V(gi, :);
    while q ~= si && q > 0
        q = prev(q);
        if q <= 0, cost = inf; way = zeros(0, size(V, 2)); return; end
        way(end + 1, :) = V(q, :);
    end
    way = flipud(way);
else
    cost = inf; way = zeros(0, size(V, 2));
end
end

function uf = uf_union(uf, a, b)
ra = uf_find(uf, a); rb = uf_find(uf, b);
if ra ~= rb, uf(rb) = ra; end
end

function r = uf_find(uf, a)
r = a;
while uf(r) ~= r, r = uf(r); end
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
