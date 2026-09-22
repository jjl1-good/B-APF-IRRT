function summary=summarize_results3d(data,outputFile)
%SUMMARIZE_RESULTS3D Aggregate raw 3-D comparison results.
if nargin<1||isempty(data),data='results_3d.csv';end
if ischar(data)||(isstring(data)&&isscalar(data)),T=readtable(data);elseif isstruct(data),T=struct2table(data);else,T=data;end
scenarios=unique(T.scenario,'stable');algorithms=unique(T.algorithm,'stable');
summary=struct('scenario',{},'algorithm',{},'runs',{},'successRate',{},'meanLength',{},'stdLength',{},'meanTime',{},'stdTime',{},'meanNodes',{});z=0;
for i=1:numel(scenarios)
 for j=1:numel(algorithms)
  m=strcmp(T.scenario,scenarios{i})&strcmp(T.algorithm,algorithms{j});A=T(m,:);ok=logical(A.success);L=A.length(ok);tt=A.time(ok);nn=A.nodes(ok);z=z+1;
  summary(z)=struct('scenario',scenarios{i},'algorithm',algorithms{j},'runs',height(A),'successRate',100*mean(ok),'meanLength',moi(L),'stdLength',soi(L),'meanTime',moi(tt),'stdTime',soi(tt),'meanNodes',moi(nn));
 end
end
summary=struct2table(summary);if nargin>=2&&~isempty(outputFile),writetable(summary,outputFile);end
end
function v=moi(x),if isempty(x),v=inf;else,v=mean(x);end,end
function v=soi(x),if numel(x)<2,v=0;else,v=std(x);end,end
