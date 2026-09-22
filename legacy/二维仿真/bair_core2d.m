function [a,t,detail] = bair_core2d(mode, opts)
%BAIR_CORE2D Reproducible BAIR pipeline with stable local guidance.
% mode is y/g/n and maps to the original fixed benchmark environment.
if nargin<2,opts=struct;end
env=bair_env2d(mode); if ~isfield(opts,'plot'),opts.plot=false;end
tic;
[apfPath,apfInfo]=stable_apf2d(env,opts);
if ~apfInfo.success, apfPath=zeros(0,2); end
if ~isfield(opts,'useLocal'),opts.useLocal=true;end
if ~isfield(opts,'useInformed'),opts.useInformed=true;end
seed=apfPath; if ~opts.useLocal,seed=[];end
if opts.useInformed, [path,rrtInfo]=informed_rrtstar2d(env,seed,opts); else, path=seed; rrtInfo=struct('success',~isempty(path),'length',path_length2d(path),'nodes',0,'iterations',0); end
if isempty(path), path=apfPath; end
a=path_length2d(path); t=toc;
detail=struct('mode',mode,'path',path,'apfPath',apfPath,'apf',apfInfo,'rrt',rrtInfo,'length',a,'time',t,'success',isfinite(a));
if opts.plot
    figure('Name',['BAIR ' upper(mode)]); hold on; axis equal; axis([env.bounds(1:2) env.bounds(3:4)]); box on;
    for k=1:size(env.square,1), rectangle('Position',env.square(k,:),'FaceColor',[.15 .15 .15],'EdgeColor','none');end
    for k=1:size(env.round,1),rectangle('Position',[env.round(k,1)-env.round(k,3),env.round(k,2)-env.round(k,3),2*env.round(k,3),2*env.round(k,3)],'Curvature',[1 1],'FaceColor',[.15 .15 .15],'EdgeColor','none');end
    plot(env.start(1),env.start(2),'mo','MarkerFaceColor','m');plot(env.goal(1),env.goal(2),'go','MarkerFaceColor','g');
    if ~isempty(apfPath),plot(apfPath(:,1),apfPath(:,2),'m--','LineWidth',1);end
    if ~isempty(path),plot(path(:,1),path(:,2),'r-','LineWidth',2);end
    legend('APF guide','BAIR path','Location','best'); title(['BAIR-' upper(mode)]);
end
end
function L=path_length2d(p),if size(p,1)<2,L=inf;else,L=sum(sqrt(sum(diff(p).^2,2)));end,end
