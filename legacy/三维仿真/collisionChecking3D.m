function feasible = collisionChecking3D(p1,p2,env)
%COLLISIONCHECKING3D Inclusive, resolution-controlled segment collision test.
feasible=false; if nargin<3||numel(p1)<3||numel(p2)<3,return;end
p1=double(p1(1:3));p2=double(p2(1:3));
if ~inside3(p1,env.bounds)||~inside3(p2,env.bounds),return;end
n=max(1,ceil(norm(p2-p1)/0.5));
for k=0:n
    p=p1+(p2-p1)*(k/n);
    for j=1:numel(env.cube.axisX)
        if p(1)>=env.cube.axisX(j)&&p(1)<=env.cube.axisX(j)+env.cube.lengthx(j)&&p(2)>=env.cube.axisY(j)&&p(2)<=env.cube.axisY(j)+env.cube.lengthy(j)&&p(3)>=env.cube.axisZ(j)&&p(3)<=env.cube.axisZ(j)+env.cube.lengthz(j),return;end
    end
    for j=1:numel(env.cylinder.X)
        if p(3)>=env.cylinder.Z(j)&&p(3)<=env.cylinder.Z(j)+env.cylinder.lengthZ(j)&&(p(1)-env.cylinder.X(j))^2+(p(2)-env.cylinder.Y(j))^2<=env.cylinder.radius(j)^2,return;end
    end
    for j=1:numel(env.sphere.X)
        if sum(([p(1)-env.sphere.X(j),p(2)-env.sphere.Y(j),p(3)-env.sphere.Z(j)]).^2)<=env.sphere.radius(j)^2,return;end
    end
end
feasible=true;
end
function ok=inside3(p,b),ok=p(1)>=b(1)&&p(1)<=b(2)&&p(2)>=b(3)&&p(2)<=b(4)&&p(3)>=b(5)&&p(3)<=b(6);end
