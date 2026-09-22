function summary = summarize_results2d(data, outputFile)
%SUMMARIZE_RESULTS2D 二维对照实验结果汇总（委托 summarize_core）。
% 输出含 Bootstrap 95% CI、效应量（Cliff's delta）与配对置换检验 p 值。
if nargin < 1 || isempty(data), data = 'results_2d.csv'; end
if ischar(data) || (isstring(data) && isscalar(data))
    T = readtable(data);
elseif isstruct(data)
    T = struct2table(data);
else
    T = data;
end
if nargin < 2, outputFile = ''; end
summary = summarize_core(T, 'AB-IRRT*', outputFile);
end
