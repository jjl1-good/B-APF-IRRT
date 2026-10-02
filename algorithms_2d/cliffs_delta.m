function [delta, mag] = cliffs_delta(x, y)
%CLIFFS_DELTA Cliff's delta 效应量（非参数，两个独立样本）。
%
%   [delta, mag] = cliffs_delta(x, y)
%
% delta ∈ [-1, 1]：x 大于 y 的概率减去 x 小于 y 的概率。
%   |delta| < 0.147            negligible
%   0.147 ≤ |delta| < 0.33     small
%   0.33  ≤ |delta| < 0.474    medium
%   |delta| ≥ 0.474            large
%
% 与 Mann-Whitney/秩和检验配套使用，用于报告“差异有多大”，
% 弥补仅报告 p 值无法反映效应大小的问题（审稿意见 R3-4）。

x = double(x(:)); x = x(isfinite(x));
y = double(y(:)); y = y(isfinite(y));
if isempty(x) || isempty(y)
    delta = NaN; mag = 'n/a'; return;
end

% 通过秩统计快速计算 delta，避免 O(n*m) 的双重循环：
% delta = (2 * (U / (n*m))) - 1，其中 U 为 Mann-Whitney U 统计量。
n = numel(x); m = numel(y);
gt = 0; lt = 0;
for i = 1:n
    di = x(i) - y;
    gt = gt + sum(di > 0);
    lt = lt + sum(di < 0);
end
delta = (gt - lt) / (n * m);

a = abs(delta);
if a < 0.147
    mag = 'negligible';
elseif a < 0.33
    mag = 'small';
elseif a < 0.474
    mag = 'medium';
else
    mag = 'large';
end
end
