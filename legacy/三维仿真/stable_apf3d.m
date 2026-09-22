function [path,info] = stable_apf3d(env,opts)
%STABLE_APF3D Bounded APF guidance with collision-safe integration.
if nargin<2,opts=struct;end
d=struct('step',3,'rho0',15,'kAtt',1,'kRep',1.1,'kTan',0.8,'maxSteps',2500,'goalTol',4,'useFallback',false);
f=fieldnames(d);for k=1:numel(f),if ~isfield(opts,f{k}),opts.(f{k})=d.(f{k});end,end
p=env.start;path=p;info=struct('success',false,'fallback',false,'iterations',0,'length',inf);stuck=0;
if collisionChecking3D(env.start,env.goal,env),path=[env.start;env.goal];info.success=true;info.iterations=1;info.length=norm(diff(path));return;end
for it=1:opts.maxSteps
    dg=norm(env.goal-p);if dg<=opts.goalTol,path(end+1,:)=env.goal;info.success=true;break;end
    fatt=(env.goal-p)/max(dg,eps);frep=[0 0 0];ftan=[0 0 0];nearest=inf;
    for j=1:numel(env.cube.axisX)
        lo=[env.cube.axisX(j),env.cube.axisY(j),env.cube.axisZ(j)];hi=lo+[env.cube.lengthx(j),env.cube.lengthy(j),env.cube.lengthz(j)];c=min(max(p,lo),hi);v=p-c;dd=norm(v);if dd<nearest,nearest=dd;end
        if dd<opts.rho0&&dd>1e-8,n=v/dd;w=min((1/dd-1/opts.rho0)/(dd^2),20);frep=frep+opts.kRep*w*n;ftan=ftan+opts.kTan*tangent3(n,env.goal-p);end
    end
    for j=1:numel(env.cylinder.X)
        cen=[env.cylinder.X(j),env.cylinder.Y(j)];rv=p(1:2)-cen;rr=norm(rv);zlo=env.cylinder.Z(j);zhi=zlo+env.cylinder.lengthZ(j);rz=min(max(p(3),zlo),zhi);q=[cen+(env.cylinder.radius(j)/max(rr,eps))*rv,rz];v=p-q;dd=norm(v);if dd<nearest,nearest=dd;end
        if dd<opts.rho0&&dd>1e-8,n=v/dd;w=min((1/dd-1/opts.rho0)/(dd^2),20);frep=frep+opts.kRep*w*n;ftan=ftan+opts.kTan*tangent3(n,env.goal-p);end
    end
    for j=1:numel(env.sphere.X)
        cen=[env.sphere.X(j),env.sphere.Y(j),env.sphere.Z(j)];v=p-cen;rr=norm(v);dd=max(rr-env.sphere.radius(j),0);if dd<nearest,nearest=dd;end
        if dd<opts.rho0&&rr>1e-8,n=v/rr;w=min((1/max(dd,1e-3)-1/opts.rho0)/(max(dd,1e-3)^2),20);frep=frep+opts.kRep*w*n;ftan=ftan+opts.kTan*tangent3(n,env.goal-p);end
    end
    beta=min(max((opts.rho0-nearest)/opts.rho0,0),1);v=fatt+beta*(frep+ftan);if norm(v)<1e-8,v=fatt;end;v=opts.step*v/max(norm(v),eps);q=p+v;
    if ~inside3(q,env.bounds)||~collisionChecking3D(p,q,env)
        q=p+0.5*opts.step*tangent3(v,[0 0 1]);
        if ~inside3(q,env.bounds)||~collisionChecking3D(p,q,env),stuck=stuck+1;else,stuck=0;end
    end
    if norm(q-p)<1e-8,stuck=stuck+1;else,p=q;path(end+1,:)=p;end
    if stuck>30,break;end
end
if ~info.success&&opts.useFallback,[path,ri]=rrtstar3d(env,opts);info.success=ri.success;info.fallback=ri.success;end
info.iterations=size(path,1)-1;if info.success,info.length=path_length3(path);end
end
function t=tangent3(n,g),t=g-dot(g,n)*n;if norm(t)<1e-8,ref=[1 0 0];if abs(dot(ref,n))>.9,ref=[0 1 0];end;t=cross(n,ref);end;t=t/max(norm(t),eps);end
function L=path_length3(p),if size(p,1)<2,L=inf;else,L=sum(sqrt(sum(diff(p).^2,2)));end,end
