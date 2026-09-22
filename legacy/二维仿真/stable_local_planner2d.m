function [path, info] = stable_local_planner2d(env, opts)
%STABLE_LOCAL_PLANNER2D Collision-safe local pre-planner.
% A* on a coarse occupancy grid is used as a stable fallback/guide for APF.
if nargin < 2, opts = struct; end
if ~isfield(opts,'gridStep'), opts.gridStep = 2; end
if ~isfield(opts,'maxNodes'), opts.maxNodes = 200000; end
step = opts.gridStep; b = env.bounds;
xs = b(1):step:b(2); ys = b(3):step:b(4);
nx = numel(xs); ny = numel(ys);
occ = false(ny,nx);
for iy=1:ny, for ix=1:nx
    occ(iy,ix) = ~collisionChecking([xs(ix) ys(iy)],[xs(ix) ys(iy)],env.squareAll,env.round);
end, end
% Keep start and goal valid even when they lie near a grid boundary.
[~,sx]=min(abs(xs-env.start(1))); [~,sy]=min(abs(ys-env.start(2)));
[~,gx]=min(abs(xs-env.goal(1)));  [~,gy]=min(abs(ys-env.goal(2)));
occ(sy,sx)=false; occ(gy,gx)=false;
g = inf(ny,nx); f = inf(ny,nx); parent = zeros(ny,nx,'uint32'); open=false(ny,nx);
g(sy,sx)=0; f(sy,sx)=hypot(gx-sx,gy-sy); open(sy,sx)=true;
dirs=[1 0;-1 0;0 1;0 -1;1 1;1 -1;-1 1;-1 -1];
found=false; expanded=0;
while any(open(:)) && expanded < opts.maxNodes
    tmp=f; tmp(~open)=inf; [~,q]=min(tmp(:)); [cy,cx]=ind2sub([ny nx],q);
    open(cy,cx)=false; expanded=expanded+1;
    if cx==gx && cy==gy, found=true; break; end
    for d=1:size(dirs,1)
        nx2=cx+dirs(d,1); ny2=cy+dirs(d,2);
        if nx2<1||nx2>nx||ny2<1||ny2>ny||occ(ny2,nx2), continue; end
        if dirs(d,1)~=0 && dirs(d,2)~=0 && (occ(cy, nx2)||occ(ny2,cx)), continue; end
        c=g(cy,cx)+norm(dirs(d,:));
        if c<g(ny2,nx2)
            g(ny2,nx2)=c; f(ny2,nx2)=c+hypot(gx-nx2,gy-ny2); parent(ny2,nx2)=uint32(q); open(ny2,nx2)=true;
        end
    end
end
if ~found
    path=zeros(0,2); info=struct('success',false,'expanded',expanded,'length',inf); return;
end
ids=zeros(1,expanded+1); k=1; q=sub2ind([ny nx],gy,gx); ids(k)=q;
while q~=sub2ind([ny nx],sy,sx)
    q=double(parent(q)); k=k+1; ids(k)=q;
end
ids=ids(k:-1:1); [py,px]=ind2sub([ny nx],ids); path=[xs(px(:))' ys(py(:))'];
path(1,:)=env.start; path(end,:)=env.goal;
% Shortcut smoothing preserves collision safety.
i=1; smooth=path(1,:);
while i<size(path,1)
    j=size(path,1);
    while j>i+1 && ~collisionChecking(smooth(end,:),path(j,:),env.squareAll,env.round), j=j-1; end
    smooth(end+1,:)=path(j,:); i=j;
end
path=smooth; info=struct('success',true,'expanded',expanded,'length',path_length2d(path));
end

function L=path_length2d(p)
if size(p,1)<2, L=0; else, L=sum(sqrt(sum(diff(p).^2,2))); end
end
