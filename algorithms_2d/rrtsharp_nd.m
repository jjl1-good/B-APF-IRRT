function [path, info] = rrtsharp_nd(bounds, start, goal, isFree, opts)
%RRTSHARP_ND  RRT#（静态情形，代价传播 + 重布线）维度通用实现。
%
%   [path, info] = rrtsharp_nd(bounds, start, goal, isFree, opts)
%
% 说明：RRTX（Mandalika & Salzman / Choudhury et al.）在静态环境下退化为
% RRT# 形式的“代价传播 + 重布线”过程。本实现遵循该静态形式：
%   1) 作为 RRT* 的扩展，在节点新增和重布线时维护子节点表；
%   2) 任何节点的父节点变化都会把代价增量传播给其整棵子树（保证树内
%      代价 g 始终与当前树结构一致，避免原 RRT* 中祖先代价陈旧的问题）；
%   3) 与标准 RRT* 相比，最优父节点选择使用“实时更新”的 g 值，等价于
%      对受影响子树做增量 Dijkstra 松弛。
%
% opts 字段（默认值）：maxIter (2000)、step (4)、goalBias (0.08)、
% rewireRadius (14)，维度通过 start 的长度自动识别。
%
% 输出 info 与其它规划器一致（含 firstSolutionTime / bestHistory 等）。

if nargin < 5, opts = struct; end
def = struct('maxIter', 2000, 'step', 4, 'goalBias', 0.08, 'rewireRadius', 14);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
start = double(start(:)'); goal = double(goal(:)');
d = numel(start);
lo = bounds(1:2:end); hi = bounds(2:2:end);
ext = hi - lo;

N = opts.maxIter + 2;
X = zeros(N, d); P = zeros(N, 1); C = inf(N, 1);
kids = cell(N, 1);                     % 子节点表（用于代价传播）
X(1, :) = start; C(1) = 0; n = 1;
best = inf; goalIdx = 0; firstSolTime = NaN;
hist = zeros(4096, 2); nh = 0; nProp = 0;
t0 = tic;

if ~isfield(opts, 'timeLimit'), opts.timeLimit = Inf; end
tWall = tic;                      % 等时间对照用：墙钟上限（默认 Inf 表示不启用）
for it = 1:opts.maxIter
    if toc(tWall) > opts.timeLimit, break; end
    if rand < opts.goalBias
        xr = goal;
    else
        xr = lo + ext .* rand(1, d);
    end
    [~, near] = min(sum((X(1:n, :) - xr).^2, 2));
    v = xr - X(near, :);
    if norm(v) < eps, continue; end
    xn = X(near, :) + min(opts.step, norm(v)) * v / norm(v);
    if ~inside_nd(xn, bounds) || ~logical(isFree(X(near, :), xn)), continue; end

    dist = sqrt(sum((X(1:n, :) - xn).^2, 2));
    cand = find(dist <= opts.rewireRadius);
    [~, ord] = sort(C(cand) + dist(cand));
    cand = cand(ord);
    par = near; cc = C(near) + norm(xn - X(near, :));
    for z = 1:numel(cand)
        j = cand(z);
        if C(j) + dist(j) < cc - 1e-12 && logical(isFree(X(j, :), xn))
            par = j; cc = C(j) + dist(j);
        end
    end

    n = n + 1;
    X(n, :) = xn; P(n) = par; C(n) = cc;
    kids{n} = zeros(0, 1);
    kids{par} = [kids{par}; n];       %#ok<AGROW>

    for z = 1:numel(cand)
        j = cand(z);
        nc = cc + norm(X(j, :) - xn);
        if j ~= par && nc < C(j) - 1e-12 && logical(isFree(xn, X(j, :)))
            delta = C(j) - nc;
            oldP = P(j);
            kids{oldP}(kids{oldP} == j) = [];
            P(j) = n; C(j) = nc;
            kids{n} = [kids{n}; j];   %#ok<AGROW>
            [C, nProp] = propagate_nd(C, kids, j, delta, nProp);
        end
    end

    dg = norm(xn - goal);
    if dg <= opts.step * 1.5 && logical(isFree(xn, goal))
        gc = cc + dg;
        if gc < best
            best = gc; goalIdx = n;
            if isnan(firstSolTime), firstSolTime = toc(t0); end
            nh = nh + 1; hist(nh, :) = [toc(t0), best];
        end
    end
end

tPlanner = toc(t0);
if goalIdx > 0
    q = goalIdx; way = goal;
    while q > 1
        way(end + 1, :) = X(q, :); %#ok<AGROW>
        q = P(q);
    end
    way(end + 1, :) = start;
    way = flipud(way);
    path = way; tShortcut = 0;   % 终路径贪心捷径后处理已取消（2026-09-20）
    L = length_nd(path);
else
    path = zeros(0, d); tShortcut = 0; L = inf;
end

hist = hist(1:nh, :);
info = struct('success', goalIdx > 0, 'length', L, 'nodes', n, 'iterations', opts.maxIter, ...
    'firstSolutionTime', firstSolTime, 'bestHistory', hist, 'propagations', nProp, ...
    'samples', X(1:n, :), 'tPlanner', tPlanner, 'tShortcut', tShortcut, 'timeTotal', toc(t0));
end

function [C, nProp] = propagate_nd(C, kids, root, delta, nProp)
% 将父节点变化产生的代价增量沿子树 BFS 传播（增量 Dijkstra 松弛）。
% 使用头指针遍历而不是反复删除队首元素，避免子树较大时出现 O(k^2) 开销。
queue = kids{root};
head = 1;
while head <= numel(queue)
    u = queue(head); head = head + 1;
    C(u) = C(u) - delta; nProp = nProp + 1;
    if ~isempty(kids{u})
        queue = [queue; kids{u}]; %#ok<AGROW>
    end
end
end

function ok = inside_nd(p, b)
ok = true;
for k = 1:numel(p)
    ok = ok && p(k) >= b(2 * k - 1) && p(k) <= b(2 * k);
end
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
