function [lo, hi, bootMean] = bootstrap_ci(x, alpha, nBoot)
%BOOTSTRAP_CI 均值的非参数 Bootstrap 置信区间（百分位法）。
%
%   [lo, hi, bootMean] = bootstrap_ci(x, alpha, nBoot)
%
% x       : 样本向量（有限值；NaN/Inf 应先剔除）。
% alpha   : 显著性水平，默认 0.05（对应 95% 置信区间）。
% nBoot   : 重采样次数，默认 10000。
%
% 返回 lo/hi 为置信区间端点，bootMean 为 B×1 的 Bootstrap 均值分布
% （可用于报告或绘图）。小样本（n<2）时返回 lo=hi=mean(x)。
%
% 该函数不依赖任何工具箱，用于替换对 Statistics Toolbox 的依赖。

if nargin < 2 || isempty(alpha), alpha = 0.05; end
if nargin < 3 || isempty(nBoot), nBoot = 10000; end

x = double(x(:));
x = x(isfinite(x));
n = numel(x);

if n < 2
    if n == 1
        lo = x; hi = x; bootMean = x;
    else
        lo = NaN; hi = NaN; bootMean = [];
    end
    return;
end

idx = randi(n, n, nBoot);
bootMean = mean(x(idx), 1)';
lo = prctile_local(bootMean, 100 * (alpha / 2));
hi = prctile_local(bootMean, 100 * (1 - alpha / 2));
end

function v = prctile_local(x, p)
% 线性插值分位数（与 MATLAB prctile 默认方式一致）。
x = sort(x(:));
n = numel(x);
if n == 0, v = NaN; return; end
rank = (p / 100) * (n - 1) + 1;      % 1-based
lo = floor(rank); hi = ceil(rank);
if lo == hi
    v = x(lo);
else
    w = rank - lo;
    v = x(lo) * (1 - w) + x(hi) * w;
end
end
