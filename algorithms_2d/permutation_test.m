function [p, stat, statNull] = permutation_test(x, y, opts)
%PERMUTATION_TEST 置换检验（两独立样本或配对样本）。
%
%   [p, stat, statNull] = permutation_test(x, y)
%   [p, stat, statNull] = permutation_test(x, y, opts)
%
% opts.paired  : true 时执行配对置换检验（默认 false）。
% opts.nPerm   : 置换次数，默认 10000。
% opts.alt     : 'two'（默认，双侧）、'greater'、'less'，相对于 x-y。
%
% 统计量：paired=true 时用配对差值的均值；否则用两组均值之差。
% p 值为置换分布下 |统计量|null| ≥ |统计量|观测 的比例（双侧），
% 单侧时按方向计算。全部实现不依赖 Statistics Toolbox。
%
% 该检验用于替代 Mann-Whitney U / Wilcoxon 检验，满足审稿意见
% R3-4 对显著性检验的要求；同时建议配合 cliffs_delta 报告效应量。

if nargin < 3, opts = struct; end
if ~isfield(opts, 'paired'), opts.paired = false; end
if ~isfield(opts, 'nPerm'), opts.nPerm = 10000; end
if ~isfield(opts, 'alt'), opts.alt = 'two'; end

x = double(x(:)); y = double(y(:));
if opts.paired
    if numel(x) ~= numel(y), error('paired permutation test needs equal-length samples'); end
    d = x - y;
    d = d(isfinite(d));
    n = numel(d);
    stat = mean(d);
    statNull = zeros(opts.nPerm, 1);
    for b = 1:opts.nPerm
        s = sign(rand(n, 1) - 0.5);   % 随机翻转符号
        statNull(b) = mean(d .* s);
    end
else
    x = x(isfinite(x)); y = y(isfinite(y));
    nx = numel(x); ny = numel(y);
    if nx < 1 || ny < 1, p = NaN; stat = NaN; statNull = []; return; end
    stat = mean(x) - mean(y);
    pool = [x; y];
    statNull = zeros(opts.nPerm, 1);
    for b = 1:opts.nPerm
        perm = randperm(nx + ny);
        statNull(b) = mean(pool(perm(1:nx))) - mean(pool(perm(nx+1:end)));
    end
end

switch lower(opts.alt)
    case 'greater'
        p = (sum(statNull >= stat) + 1) / (opts.nPerm + 1);
    case 'less'
        p = (sum(statNull <= stat) + 1) / (opts.nPerm + 1);
    otherwise
        p = (sum(abs(statNull) >= abs(stat)) + 1) / (opts.nPerm + 1);
end
end
