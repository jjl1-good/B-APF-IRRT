function summary = summarize_results2d(data, outputFile)
%SUMMARIZE_RESULTS2D Aggregate raw 2-D comparison results for reporting.
if nargin<1||isempty(data), data='results_2d.csv'; end
if ischar(data) || (isstring(data) && isscalar(data)), T=readtable(data); elseif isstruct(data), T=struct2table(data); else, T=data; end
scenarios=unique(T.scenario,'stable'); algorithms=unique(T.algorithm,'stable');
summary=struct('scenario',{},'algorithm',{},'runs',{},'successRate',{}, ...
    'meanLength',{},'stdLength',{},'meanTime',{},'stdTime',{},'meanNodes',{}); z=0;
for i=1:numel(scenarios), for j=1:numel(algorithms)
    mask=strcmp(T.scenario,scenarios{i})&strcmp(T.algorithm,algorithms{j}); A=T(mask,:); ok=logical(A.success);
    L=A.length(ok); tt=A.time(ok); nn=A.nodes(ok); z=z+1;
    summary(z)=struct('scenario',scenarios{i},'algorithm',algorithms{j},'runs',height(A), ...
        'successRate',100*mean(ok),'meanLength',mean_or_inf(L),'stdLength',std_or_zero(L), ...
        'meanTime',mean_or_inf(tt),'stdTime',std_or_zero(tt),'meanNodes',mean_or_inf(nn));
end,end
summary=struct2table(summary);
if nargin>=2&&~isempty(outputFile), writetable(summary,outputFile); end
end
function v=mean_or_inf(x),if isempty(x),v=inf;else,v=mean(x);end,end
function v=std_or_zero(x),if numel(x)<2,v=0;else,v=std(x);end,end
