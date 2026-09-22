function [path, info] = stable_apf2d(env, opts)
%STABLE_APF2D Adaptive APF with bounded repulsion/tangential influence.
if nargin<2, opts=struct; end
defaults=struct('step',2.0,'rho0',12,'kAtt',1.0,'kRep',1.2,'kTan',0.9,'maxSteps',2500,'goalTol',3,'useFallback',true);
fn=fieldnames(defaults); for k=1:numel(fn), if ~isfield(opts,fn{k}), opts.(fn{k})=defaults.(fn{k}); end, end
p=env.start; path=p; stagn=0; info=struct('success',false,'fallback',false,'iterations',0,'length',inf);
for it=1:opts.maxSteps
    dg=norm(env.goal-p); if dg<=opts.goalTol, path(end+1,:)=env.goal; info.success=true; break; end
    fatt=(env.goal-p)/max(dg,eps); frep=[0 0]; ftan=[0 0]; nearest=inf;
    obs=[env.squareAll(:,1:2),env.squareAll(:,1)+env.squareAll(:,3),env.squareAll(:,2)+env.squareAll(:,4)];
    for j=1:size(obs,1)
        c=[min(max(p(1),obs(j,1)),obs(j,3)),min(max(p(2),obs(j,2)),obs(j,4))]; v=p-c; d=norm(v);
        if d<nearest, nearest=d; end
        if d<opts.rho0 && d>1e-6
            n=v/d; w=(1/d-1/opts.rho0)/(d^2); frep=frep+opts.kRep*min(w,20)*n;
            t=[-n(2) n(1)]; s=sign(t(1)*(env.goal(2)-p(2))-t(2)*(env.goal(1)-p(1))); if s==0,s=1;end
            ftan=ftan+opts.kTan*(1-d/opts.rho0)*s*t;
        end
    end
    % Smoothly blend attraction, repulsion and tangent; never discard terms.
    beta=min(max((opts.rho0-nearest)/opts.rho0,0),1); v=fatt+beta*(frep+ftan);
    if norm(v)<1e-6, v=fatt+0.25*ftan; end
    v=v/max(norm(v),eps)*opts.step; q=p+v;
    if q(1)<env.bounds(1)||q(1)>env.bounds(2)||q(2)<env.bounds(3)||q(2)>env.bounds(4)||~collisionChecking(p,q,env.squareAll,env.round)
        % Rotate tangentially and try a shorter safe step.
        q=p+opts.step*0.5*[-v(2) v(1)]/max(norm(v),eps);
        if q(1)<env.bounds(1)||q(1)>env.bounds(2)||q(2)<env.bounds(3)||q(2)>env.bounds(4)||~collisionChecking(p,q,env.squareAll,env.round), stagn=stagn+1; else, stagn=0; end
    end
    if norm(q-p)<1e-8, stagn=stagn+1; else, p=q; path(end+1,:)=p; end
    if stagn>25, break; end
end
if ~info.success && opts.useFallback
    [fallback,fi]=stable_local_planner2d(env,opts); if fi.success, path=fallback; info.success=true; info.fallback=true; end
end
info.iterations=size(path,1)-1; info.length=path_length2d(path);
if ~info.success, info.length=inf; end
end
function L=path_length2d(p), if size(p,1)<2,L=inf;else,L=sum(sqrt(sum(diff(p).^2,2)));end,end
