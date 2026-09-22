function [path, info] = batch_rrtstar2d(env, opts)
%BATCH_RRTSTAR2D Batched uniform-sampling RRT* baseline.
% Samples are generated in batches, ordered by a lower-bound cost and then
% inserted with the same collision-safe parent selection/rewiring rule.
if nargin < 2, opts = struct; end
if ~isfield(opts,'batchSize'), opts.batchSize = 200; end
if ~isfield(opts,'batches'), opts.batches = 20; end
if ~isfield(opts,'step'), opts.step = 5; end
if ~isfield(opts,'rewireRadius'), opts.rewireRadius = 15; end
if ~isfield(opts,'goalBias'), opts.goalBias = 0.08; end
N=opts.batchSize*opts.batches+2; X=zeros(N,2); P=zeros(N,1); C=inf(N,1);
X(1,:)=env.start; C(1)=0; n=1; goalIdx=0; best=inf;
for batch=1:opts.batches
    S=zeros(opts.batchSize,2);
    for k=1:opts.batchSize
        if rand<opts.goalBias, S(k,:)=env.goal; else
            S(k,:)=[env.bounds(1)+(env.bounds(2)-env.bounds(1))*rand, ...
                env.bounds(3)+(env.bounds(4)-env.bounds(3))*rand];
        end
    end
    score=sum((S-env.start).^2,2)+sum((S-env.goal).^2,2); [~,ord]=sort(score); S=S(ord,:);
    for k=1:size(S,1)
        xr=S(k,:); [~,near]=min(sum((X(1:n,:)-xr).^2,2)); d=xr-X(near,:);
        if norm(d)<eps, continue; end
        xn=X(near,:)+min(opts.step,norm(d))*d/norm(d);
        if ~inside(xn,env.bounds)||~collisionChecking(X(near,:),xn,env.squareAll,env.round),continue;end
        dist=sqrt(sum((X(1:n,:)-xn).^2,2)); cand=find(dist<=opts.rewireRadius);
        par=near; cc=C(near)+norm(xn-X(near,:));
        for z=1:numel(cand)
            j=cand(z); if C(j)+dist(j)<cc && collisionChecking(X(j,:),xn,env.squareAll,env.round),par=j;cc=C(j)+dist(j);end,end
        n=n+1;X(n,:)=xn;P(n)=par;C(n)=cc;
        for z=1:numel(cand)
            j=cand(z); nc=cc+norm(X(j,:)-xn);
            if j~=par&&nc<C(j)&&collisionChecking(xn,X(j,:),env.squareAll,env.round),P(j)=n;C(j)=nc;end
        end
        dg=norm(xn-env.goal); if dg<=opts.step*1.5&&collisionChecking(xn,env.goal,env.squareAll,env.round)
            gc=cc+dg; if gc<best,best=gc;goalIdx=n;end
        end
    end
end
if goalIdx>0
    q=goalIdx;path=env.goal;while q>1,path(end+1,:)=X(q,:);q=P(q);end;path(end+1,:)=env.start;path=flipud(path);path=shortcut(path,env);
else,path=zeros(0,2);end
info=struct('success',goalIdx>0,'length',path_length2d(path),'nodes',n, ...
    'iterations',opts.batchSize*opts.batches,'batches',opts.batches,'goalFound',goalIdx>0);
end
function ok=inside(x,b),ok=x(1)>=b(1)&&x(1)<=b(2)&&x(2)>=b(3)&&x(2)<=b(4);end
function p=shortcut(p,e)
if size(p,1)<3,return;end;i=1;while i<size(p,1)-1,j=size(p,1);while j>i+1&&~collisionChecking(p(i,:),p(j,:),e.squareAll,e.round),j=j-1;end;p=[p(1:i,:);p(j:end,:)];i=i+1;end
end
function L=path_length2d(p),if size(p,1)<2,L=inf;else,L=sum(sqrt(sum(diff(p).^2,2)));end,end
