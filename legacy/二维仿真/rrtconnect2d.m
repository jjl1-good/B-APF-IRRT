function [path, info] = rrtconnect2d(env, opts)
%RRTCONNECT2D Bidirectional RRT-Connect baseline.
if nargin < 2, opts = struct; end
if ~isfield(opts,'maxIter'), opts.maxIter = 4000; end
if ~isfield(opts,'step'), opts.step = 5; end
if ~isfield(opts,'goalBias'), opts.goalBias = 0.1; end
Xa=env.start; Pa=0; Xb=env.goal; Pb=0; found=false; ia=0; ib=0;
for it=1:opts.maxIter
    if rand<opts.goalBias, xr=env.goal; else, xr=[env.bounds(1)+(env.bounds(2)-env.bounds(1))*rand,env.bounds(3)+(env.bounds(4)-env.bounds(3))*rand]; end
    [Xa,Pa,ia,ok]=extend(Xa,Pa,xr,env,opts.step); if ~ok, [Xa,Xb]=swap(Xa,Xb); [Pa,Pb]=swap(Pa,Pb); continue; end
    [Xb,Pb,ib,met]=connect(Xb,Pb,Xa(ia,:),env,opts.step);
    if met, found=true; break; end
    [Xa,Xb]=swap(Xa,Xb); [Pa,Pb]=swap(Pa,Pb);
end
if found
    pa=tracePath(Xa,Pa,ia); pb=tracePath(Xb,Pb,ib);
    if norm(pa(end,:)-pb(end,:))<1e-9, pb=flipud(pb); else, pa=flipud(pa); end
    path=[pa;pb(2:end,:)]; path=shortcut(path,env);
else
    path=zeros(0,2);
end
info=struct('success',found,'length',path_length2d(path), ...
    'nodes',size(Xa,1)+size(Xb,1),'iterations',it,'goalFound',found);
end
function [X,P,idx,ok]=extend(X,P,target,e,step)
[~,idx]=min(sum((X-target).^2,2)); d=target-X(idx,:); ok=false;
if norm(d)<eps, return; end
q=X(idx,:)+min(step,norm(d))*d/norm(d);
if ~inside(q,e.bounds)||~collisionChecking(X(idx,:),q,e.squareAll,e.round), return; end
X(end+1,:)=q; P(end+1)=idx; idx=size(X,1); ok=true;
end
function [X,P,idx,met]=connect(X,P,target,e,step)
met=false; idx=0;
while true
    [X,P,idx,ok]=extend(X,P,target,e,step); if ~ok, return; end
    if norm(X(idx,:)-target)<=step*1.01
        % Attach the exact target node so the two reconstructed paths join.
        if norm(X(idx,:)-target)>1e-9 && collisionChecking(X(idx,:),target,e.squareAll,e.round)
            X(end+1,:)=target; P(end+1)=idx; idx=size(X,1);
        end
        met=true; return;
    end
end
end
function p=tracePath(X,P,idx)
p=X(idx,:); while idx>1, idx=P(idx); p(end+1,:)=X(idx,:); end; p=flipud(p);
end
function [a,b]=swap(a,b), c=a; a=b; b=c; end
function ok=inside(x,b), ok=x(1)>=b(1)&&x(1)<=b(2)&&x(2)>=b(3)&&x(2)<=b(4); end
function p=shortcut(p,e)
if size(p,1)<3, return; end
i=1; while i<size(p,1)-1, j=size(p,1); while j>i+1&&~collisionChecking(p(i,:),p(j,:),e.squareAll,e.round),j=j-1;end;p=[p(1:i,:);p(j:end,:)];i=i+1;end
end
function L=path_length2d(p), if size(p,1)<2,L=inf;else,L=sum(sqrt(sum(diff(p).^2,2)));end,end
