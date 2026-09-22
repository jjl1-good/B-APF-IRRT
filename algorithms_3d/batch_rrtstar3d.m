function [path,info]=batch_rrtstar3d(env,opts)
%BATCH_RRTSTAR3D Batched uniform-sampling 3-D RRT* baseline.
if nargin<2,opts=struct;end;if ~isfield(opts,'batchSize'),opts.batchSize=150;end;if ~isfield(opts,'batches'),opts.batches=15;end;if ~isfield(opts,'step'),opts.step=6;end;if ~isfield(opts,'rewireRadius'),opts.rewireRadius=18;end;if ~isfield(opts,'goalBias'),opts.goalBias=.08;end
N=opts.batchSize*opts.batches+2;X=zeros(N,3);P=zeros(N,1);C=inf(N,1);X(1,:)=env.start;C(1)=0;n=1;goalIdx=0;best=inf;
for batch=1:opts.batches
 S=zeros(opts.batchSize,3);for k=1:opts.batchSize,if rand<opts.goalBias,S(k,:)=env.goal;else,S(k,:)=[env.bounds(1)+(env.bounds(2)-env.bounds(1))*rand,env.bounds(3)+(env.bounds(4)-env.bounds(3))*rand,env.bounds(5)+(env.bounds(6)-env.bounds(5))*rand];end,end
 [~,ord]=sort(sum((S-env.start).^2,2)+sum((S-env.goal).^2,2));S=S(ord,:);
 for k=1:size(S,1)
  xr=S(k,:);[~,near]=min(sum((X(1:n,:)-xr).^2,2));v=xr-X(near,:);if norm(v)<eps,continue;end;xn=X(near,:)+min(opts.step,norm(v))*v/norm(v);if ~inside3(xn,env.bounds)||~collisionChecking3D(X(near,:),xn,env),continue;end
  dist=sqrt(sum((X(1:n,:)-xn).^2,2));cand=find(dist<=opts.rewireRadius);par=near;cc=C(near)+norm(xn-X(near,:));for z=1:numel(cand),j=cand(z);if C(j)+dist(j)<cc&&collisionChecking3D(X(j,:),xn,env),par=j;cc=C(j)+dist(j);end,end;n=n+1;X(n,:)=xn;P(n)=par;C(n)=cc;for z=1:numel(cand),j=cand(z);nc=cc+norm(X(j,:)-xn);if j~=par&&nc<C(j)&&collisionChecking3D(xn,X(j,:),env),P(j)=n;C(j)=nc;end,end
  dg=norm(xn-env.goal);if dg<=opts.step*1.5&&collisionChecking3D(xn,env.goal,env),gc=cc+dg;if gc<best,best=gc;goalIdx=n;end,end
 end
end
if goalIdx>0,q=goalIdx;path=env.goal;while q>1,path(end+1,:)=X(q,:);q=P(q);end;path(end+1,:)=env.start;path=flipud(path);else,path=zeros(0,3);end
info=struct('success',goalIdx>0,'length',path_length3(path),'nodes',n,'iterations',opts.batchSize*opts.batches,'batches',opts.batches,'goalFound',goalIdx>0);
end
function ok=inside3(p,b),ok=p(1)>=b(1)&&p(1)<=b(2)&&p(2)>=b(3)&&p(2)<=b(4)&&p(3)>=b(5)&&p(3)<=b(6);end
function L=path_length3(p),if size(p,1)<2,L=inf;else,L=sum(sqrt(sum(diff(p).^2,2)));end,end
