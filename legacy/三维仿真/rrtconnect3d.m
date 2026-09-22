function [path,info]=rrtconnect3d(env,opts)
%RRTCONNECT3D Bidirectional 3-D RRT-Connect baseline.
if nargin<2,opts=struct;end;if ~isfield(opts,'maxIter'),opts.maxIter=4000;end;if ~isfield(opts,'step'),opts.step=7;end
Xa=env.start;Pa=0;Xb=env.goal;Pb=0;found=false;ia=0;ib=0;
for it=1:opts.maxIter
 xr=[env.bounds(1)+(env.bounds(2)-env.bounds(1))*rand,env.bounds(3)+(env.bounds(4)-env.bounds(3))*rand,env.bounds(5)+(env.bounds(6)-env.bounds(5))*rand];
 [Xa,Pa,ia,ok]=extend3(Xa,Pa,xr,env,opts.step);if ~ok,[Xa,Xb]=swap3(Xa,Xb);[Pa,Pb]=swap3(Pa,Pb);continue;end
 [Xb,Pb,ib,met]=connect3(Xb,Pb,Xa(ia,:),env,opts.step);if met,found=true;break;end;[Xa,Xb]=swap3(Xa,Xb);[Pa,Pb]=swap3(Pa,Pb);
end
if found,pa=trace3(Xa,Pa,ia);pb=flipud(trace3(Xb,Pb,ib));path=shortcut3([pa;pb(2:end,:)],env);else,path=zeros(0,3);end
info=struct('success',found,'length',path_length3(path),'nodes',size(Xa,1)+size(Xb,1),'iterations',it,'goalFound',found);
end
function [X,P,idx,ok]=extend3(X,P,target,e,step),[~,idx]=min(sum((X-target).^2,2));v=target-X(idx,:);ok=false;if norm(v)<eps,return;end;q=X(idx,:)+min(step,norm(v))*v/norm(v);if ~inside3(q,e.bounds)||~collisionChecking3D(X(idx,:),q,e),return;end;X(end+1,:)=q;P(end+1)=idx;idx=size(X,1);ok=true;end
function [X,P,idx,met]=connect3(X,P,target,e,step),met=false;idx=0;while true,[X,P,idx,ok]=extend3(X,P,target,e,step);if ~ok,return;end;if norm(X(idx,:)-target)<=step*1.01,if norm(X(idx,:)-target)>1e-9,X(end+1,:)=target;P(end+1)=idx;idx=size(X,1);end;met=true;return;end,end,end
function p=trace3(X,P,idx),p=X(idx,:);while idx>1,idx=P(idx);p(end+1,:)=X(idx,:);end;p=flipud(p);end
function [a,b]=swap3(a,b),c=a;a=b;b=c;end
function ok=inside3(p,b),ok=p(1)>=b(1)&&p(1)<=b(2)&&p(2)>=b(3)&&p(2)<=b(4)&&p(3)>=b(5)&&p(3)<=b(6);end
function p=shortcut3(p,e),if size(p,1)<3,return;end;i=1;while i<size(p,1)-1,j=size(p,1);while j>i+1&&~collisionChecking3D(p(i,:),p(j,:),e),j=j-1;end;p=[p(1:i,:);p(j:end,:)];i=i+1;end,end
function L=path_length3(p),if size(p,1)<2,L=inf;else,L=sum(sqrt(sum(diff(p).^2,2)));end,end
