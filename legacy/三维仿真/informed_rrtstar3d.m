function [path,info]=informed_rrtstar3d(env,seedPath,opts)
%INFORMED_RRTSTAR3D Informed 3-D RRT* with optional APF seed.
if nargin<2,seedPath=[];end;if nargin<3,opts=struct;end
if ~isfield(opts,'maxIter'),opts.maxIter=6000;end;if ~isfield(opts,'step'),opts.step=6;end;if ~isfield(opts,'goalBias'),opts.goalBias=.12;end;if ~isfield(opts,'rewireRadius'),opts.rewireRadius=18;end
N=opts.maxIter+2;X=zeros(N,3);P=zeros(N,1);C=inf(N,1);X(1,:)=env.start;C(1)=0;n=1;goalIdx=0;best=inf;if ~isempty(seedPath),best=path_length3(seedPath);end
for it=1:opts.maxIter
    if rand<opts.goalBias,xr=env.goal;elseif isfinite(best),xr=sample3(env.start,env.goal,best,env.bounds);else,xr=[randb(env.bounds(1),env.bounds(2)),randb(env.bounds(3),env.bounds(4)),randb(env.bounds(5),env.bounds(6))];end
    [~,near]=min(sum((X(1:n,:)-xr).^2,2));v=xr-X(near,:);if norm(v)<eps,continue;end;xn=X(near,:)+min(opts.step,norm(v))*v/norm(v);
    if ~inside3(xn,env.bounds)||~collisionChecking3D(X(near,:),xn,env),continue;end
    dist=sqrt(sum((X(1:n,:)-xn).^2,2));cand=find(dist<=opts.rewireRadius);par=near;cc=C(near)+norm(xn-X(near,:));
    for z=1:numel(cand),j=cand(z);if C(j)+dist(j)<cc&&collisionChecking3D(X(j,:),xn,env),par=j;cc=C(j)+dist(j);end,end
    n=n+1;X(n,:)=xn;P(n)=par;C(n)=cc;
    for z=1:numel(cand),j=cand(z);nc=cc+norm(X(j,:)-xn);if j~=par&&nc<C(j)&&collisionChecking3D(xn,X(j,:),env),P(j)=n;C(j)=nc;end,end
    dg=norm(xn-env.goal);if dg<=opts.step*1.5&&collisionChecking3D(xn,env.goal,env),gc=cc+dg;if gc<best,best=gc;goalIdx=n;end,end
end
if goalIdx>0,q=goalIdx;path=env.goal;while q>1,path(end+1,:)=X(q,:);q=P(q);end;path(end+1,:)=env.start;path=flipud(path);path=shortcut3(path,env);elseif ~isempty(seedPath),path=seedPath;else,path=zeros(0,3);end
info=struct('success',~isempty(path),'length',path_length3(path),'nodes',n,'iterations',opts.maxIter,'goalFound',goalIdx>0);
end
function xr=sample3(s,g,c,b),d=norm(g-s);if c<=d,xr=[randb(b(1),b(2)),randb(b(3),b(4)),randb(b(5),b(6))];return;end;a=c/2;bb=sqrt(max(c^2-d^2,eps))/2;r=rand^(1/3);u=randn(1,3);u=u/norm(u);z=[a*r*u(1),bb*r*u(2),bb*r*u(3)];e=(g-s)/d;ref=[0 0 1];if abs(dot(e,ref))>.9,ref=[0 1 0];end;v=cross(e,ref);v=v/norm(v);w=cross(e,v);xr=(s+g)/2+z(1)*e+z(2)*v+z(3)*w;end
function x=randb(a,b),x=a+(b-a)*rand;end
function ok=inside3(p,b),ok=p(1)>=b(1)&&p(1)<=b(2)&&p(2)>=b(3)&&p(2)<=b(4)&&p(3)>=b(5)&&p(3)<=b(6);end
function p=shortcut3(p,e),if size(p,1)<3,return;end;i=1;while i<size(p,1)-1,j=size(p,1);while j>i+1&&~collisionChecking3D(p(i,:),p(j,:),e),j=j-1;end;p=[p(1:i,:);p(j:end,:)];i=i+1;end,end
function L=path_length3(p),if size(p,1)<2,L=inf;else,L=sum(sqrt(sum(diff(p).^2,2)));end,end
