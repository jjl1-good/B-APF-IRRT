function [L,T,detail]=bair_core3d(mode,opts)
%BAIR_CORE3D Unified 3-D BAIR pipeline with fixed original maps.
if nargin<2,opts=struct;end;if ~isfield(opts,'useFallback'),opts.useFallback=true;end;if ~isfield(opts,'plot'),opts.plot=false;end
env=bair_env3d(mode);tic;ao=opts;ao.useFallback=opts.useFallback;[guide,ai]=stable_apf3d(env,ao);seed=guide;if ~ai.success,seed=[];end;[path,ri]=informed_rrtstar3d(env,seed,opts);if isempty(path),path=guide;end;L=path_length3(path);T=toc;detail=struct('mode',mode,'path',path,'apfPath',guide,'apf',ai,'rrt',ri,'length',L,'time',T,'success',isfinite(L));
if opts.plot&&~isempty(path),figure;plot3(path(:,1),path(:,2),path(:,3),'r-','LineWidth',2);grid on;axis equal;axis([env.bounds(1:2),env.bounds(3:4),env.bounds(5:6)]);title(['BAIR-3D ' mode]);end
end
function L=path_length3(p),if size(p,1)<2,L=inf;else,L=sum(sqrt(sum(diff(p).^2,2)));end,end
