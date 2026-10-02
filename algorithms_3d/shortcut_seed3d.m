function [p, applied, L0, L1] = shortcut_seed3d(p, env, ratioLimit)
%SHORTCUT_SEED3D 三维版：对预规划种子路径做一次确定性贪心捷径化。
% 规则与 shortcut_seed2d 相同：仅当长度 > ratioLimit × ‖goal − start‖ 时才执行。
% 三维原有场景的原始种子可达欧氏下界的 2.5 倍（general 716.4），收紧上界收益最大。

if nargin < 3 || isempty(ratioLimit), ratioLimit = 1.15; end
applied = false;
if isempty(p) || size(p, 1) < 3
    L0 = path_length3(p); L1 = L0;
    return;
end

d = norm(env.goal(:)' - env.start(:)');
L0 = path_length3(p);
if ~(L0 > ratioLimit * d)
    L1 = L0;
    return;
end

q = p;
i = 1;
while i < size(q, 1) - 1
    j = size(q, 1);
    while j > i + 1 && ~collisionChecking3D(q(i, :), q(j, :), env)
        j = j - 1;
    end
    q = [q(1:i, :); q(j:end, :)]; %#ok<AGROW>
    i = i + 1;
end
L1 = path_length3(q);
if L1 < L0 - 1e-9
    p = q; applied = true;
else
    L1 = L0;
end
end

function L = path_length3(p)
if isempty(p) || size(p, 1) < 2
    L = inf;
else
    L = sum(sqrt(sum(diff(p).^2, 2)));
end
end
