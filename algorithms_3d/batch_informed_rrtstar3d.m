function [path,info]=batch_informed_rrtstar3d(env,seedPath,opts)
%BATCH_INFORMED_RRTSTAR3D Batched informed RRT* baseline.
if nargin<2,seedPath=[];end;if nargin<3,opts=struct;end;if ~isfield(opts,'batchSize'),opts.batchSize=150;end;if ~isfield(opts,'batches'),opts.batches=15;end;if ~isfield(opts,'step'),opts.step=6;end
best=inf;path=seedPath;if ~isempty(seedPath),best=path_length3(seedPath);end;total=0;nodes=0;
for b=1:opts.batches,ro=opts;ro.maxIter=opts.batchSize;ro.goalBias=.15;ro.rewireRadius=opts.step*3;[cand,ci]=informed_rrtstar3d(env,path,ro);total=total+ci.iterations;nodes=nodes+ci.nodes;if ci.success&&ci.length<best,best=ci.length;path=cand;end,end
info=struct('success',~isempty(path),'length',path_length3(path),'nodes',nodes,'iterations',total,'batches',opts.batches);
end
function L=path_length3(p),if size(p,1)<2,L=inf;else,L=sum(sqrt(sum(diff(p).^2,2)));end,end
