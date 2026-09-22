function [p, applied, L0, L1] = shortcut_seed2d(p, env, ratioLimit)
%SHORTCUT_SEED2D 对预规划种子路径做一次确定性贪心捷径化（无碰撞校验后的直连替换）。
%
% 动机：反应式规划器给出的折线常带有无谓绕行（二维广阔环境达欧氏直线的 1.50 倍）。
% 该路径同时被用作（a）代价上界 c_best →（b）知情采样椭圆的初始大小 →（c）采样偏置目标；
% 上界越松，椭圆越接近整幅工作空间，"快速锁定采样区域"的优势就越晚出现。
% 因此在上界被使用之前先做一次捷径化，把它收紧。
%
% 规则：仅当路径长度 > ratioLimit × ‖goal − start‖ 时才执行（默认 1.15）。
%       捷径化本身是确定性、逐段碰撞校验的贪心直连，保证路径仍然无碰撞。
%
% 输入：p（N×2 折线）、env（含 squareAll/round/start/goal）、ratioLimit（默认 1.15）
% 输出：p（可能被替换）、applied（是否执行了替换）、L0/L1（执行前后长度）

if nargin < 3 || isempty(ratioLimit), ratioLimit = 1.15; end
applied = false;
if isempty(p) || size(p, 1) < 3
    L0 = path_length2d(p); L1 = L0;
    return;
end

d = norm(env.goal(:)' - env.start(:)');
L0 = path_length2d(p);
if ~(L0 > ratioLimit * d)
    L1 = L0;
    return;                      % 已经很紧（例如栅格 A* 回退路径之外的近直线解），不做处理
end

q = p;
i = 1;
while i < size(q, 1) - 1
    j = size(q, 1);
    while j > i + 1 && ~collisionChecking(q(i, :), q(j, :), env.squareAll, env.round)
        j = j - 1;
    end
    q = [q(1:i, :); q(j:end, :)]; %#ok<AGROW>
    i = i + 1;
end
L1 = path_length2d(q);
if L1 < L0 - 1e-9
    p = q; applied = true;
else
    L1 = L0;
end
end

function L = path_length2d(p)
if isempty(p) || size(p, 1) < 2
    L = inf;
else
    L = sum(sqrt(sum(diff(p).^2, 2)));
end
end
