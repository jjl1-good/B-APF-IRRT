function [path, info] = informed_rrtstar2d(env, seedPath, opts)
%INFORMED_RRTSTAR2D Collision-safe informed RRT* optimizer.
if nargin<2,seedPath=[];end; if nargin<3,opts=struct;end
if ~isfield(opts,'maxIter'),opts.maxIter=6000;end; if ~isfield(opts,'step'),opts.step=4;end
if ~isfield(opts,'goalBias'),opts.goalBias=0.12;end; if ~isfield(opts,'rewireRadius'),opts.rewireRadius=14;end
N=opts.maxIter+2; X=zeros(N,2); P=zeros(N,1); C=inf(N,1); X(1,:)=env.start; C(1)=0; n=1; best=inf; goalIdx=0;
if ~isempty(seedPath), best=path_length2d(seedPath); end
for it=1:opts.maxIter
    if rand<opts.goalBias, xr=env.goal; elseif isfinite(best), xr=sample_ellipse(env.start,env.goal,best,env.bounds); else, xr=[rand_range(env.bounds(1),env.bounds(2)),rand_range(env.bounds(3),env.bounds(4))]; end
    [~,near]=min(sum((X(1:n,:)-xr).^2,2)); d=xr-X(near,:); if norm(d)<eps,continue;end; xn=X(near,:)+min(opts.step,norm(d))*d/norm(d);
    if ~inside(xn,env.bounds)||~collisionChecking(X(near,:),xn,env.squareAll,env.round),continue;end
    dist=sqrt(sum((X(1:n,:)-xn).^2,2)); cand=find(dist<=opts.rewireRadius); [~,ord]=sort(C(cand)+dist(cand)); cand=cand(ord); par=near; cc=C(near)+norm(xn-X(near,:));
    for z=1:numel(cand), j=cand(z); if C(j)+dist(j)<cc && collisionChecking(X(j,:),xn,env.squareAll,env.round),par=j;cc=C(j)+dist(j);end,end
    n=n+1;X(n,:)=xn;P(n)=par;C(n)=cc;
    for z=1:numel(cand), j=cand(z); if j~=par && cc+norm(X(j,:)-xn)<C(j) && collisionChecking(xn,X(j,:),env.squareAll,env.round),P(j)=n;C(j)=cc+norm(X(j,:)-xn);end,end
    dg=norm(xn-env.goal); if dg<opts.step*1.5 && collisionChecking(xn,env.goal,env.squareAll,env.round)
        gc=cc+dg; if gc<best, best=gc; goalIdx=n; end
    end
end
if goalIdx>0
    q=goalIdx; path=env.goal; while q>1,path(end+1,:)=X(q,:);q=P(q);end;path(end+1,:)=env.start;path=flipud(path);path=shortcut(path,env); 
elseif ~isempty(seedPath), path=seedPath;
else, path=zeros(0,2); end
info=struct('success',~isempty(path),'length',path_length2d(path),'nodes',n,'iterations',opts.maxIter,'goalFound',goalIdx>0);
end
function xr=sample_ellipse(s,g,c,bounds)
a=c/2; d=norm(g-s);
if c<=d, xr=[rand_range(bounds(1),bounds(2)),rand_range(bounds(3),bounds(4))]; return; end
b=sqrt(max(c^2-d^2,eps))/2; u=sqrt(rand); t=2*pi*rand;
p=[a*u*cos(t) b*u*sin(t)]; th=atan2(g(2)-s(2),g(1)-s(1));
xr=p*[cos(th) -sin(th);sin(th) cos(th)]+(s+g)/2;
end
function x=rand_range(a,b),x=a+(b-a)*rand;end
function ok=inside(x,b),ok=x(1)>=b(1)&&x(1)<=b(2)&&x(2)>=b(3)&&x(2)<=b(4);end
function p=shortcut(p,e),if size(p,1)<3,return;end;i=1;while i<size(p,1)-1,j=size(p,1);while j>i+1&&~collisionChecking(p(i,:),p(j,:),e.squareAll,e.round),j=j-1;end;p=[p(1:i,:);p(j:end,:)];i=i+1;end,end
function L=path_length2d(p),if size(p,1)<2,L=inf;else,L=sum(sqrt(sum(diff(p).^2,2)));end,end
