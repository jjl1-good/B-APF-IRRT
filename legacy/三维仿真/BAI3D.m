%%清空变量
clear all
clc
tic

%%定义变量
axisStart = [0 0 0];
axisLWH=[150 150 150];

%%定义障碍物
cubeInfo.exist=0;
cylinderInfo.exist=0;
sphereInfo.exist=0;


%%定义初始点
pathPoint= [1 1 1;
            150 150 150];  %路径点
update_count = 0;      
cubeInfo = createCube(cubeInfo);  %长方体障碍物信息
cylinderInfo = createcylinder(cylinderInfo);  %圆柱障碍物信息
sphereInfo =createSphere(sphereInfo);   %球体障碍物信息
influencerange = 10 ;

%%画图
figure(1);
colorMatCube = [1 0 0]; %长方体障碍颜色
colorMatCylinder = [0 1 0];%圆柱障碍颜色
colorMatSphere = [0 0 1];%球体障碍颜色
pellucidity = 0.1 ;%透明度
hold on;
scatter3(pathPoint(1,1),pathPoint(1,2),pathPoint(1,3),'MarkerEdgeColor','k','MarkerFaceColor',[1 0 0]);
scatter3(pathPoint(2,1),pathPoint(2,2),pathPoint(2,3),'MarkerEdgeColor','k','MarkerFaceColor','b');
drawCube(cubeInfo,colorMatCube,pellucidity); %%画长方体
drawcylinder(cylinderInfo,colorMatCylinder,pellucidity);%%画圆柱
drawsphere(sphereInfo,colorMatSphere,pellucidity);%%画球体
text(pathPoint(1,1),pathPoint(1,2),pathPoint(1,3),'起点');
text(pathPoint(2,1),pathPoint(2,2),pathPoint(2,3),'终点');
view(3)
grid on;
axis equal;
axis([0 150 0 150 0 150]);
xlabel('x')
ylabel('y')
zlabel('z')

%构建障碍物影响范围(由于可能出现的遮挡问题不绘制了）
totalPath=[];
k1=1;
startPoint = pathPoint(1,:);%确定起点
goalPoint = pathPoint(2,:);%确定终点
Thr = 2 ;%范围阈值
k_att=0.001;         %设置引力常数
k_rtex = 0.002;   %设置涡流力常数
k_cutex = 0.002;   %设置方形障碍物涡流力常数
k_cytex = 0.002;   %设置柱形障碍物涡流力常数
k_rep=0.9;        %设置斥力常数
r_f=50;              %设置引力影响变化范围
%开始BUG-APF寻路
i=1;
maxSteps = 5000;
b(maxSteps) = struct('x', 0, 'y', 0, 'z', 0, 'dx', 0, 'dy', 0, 'dz', 0);
b(i).x=startPoint(1) ;
b(i).y=startPoint(2) ;
b(i).z=startPoint(3) ;
b(i).dx=0;
b(i).dy=0;
b(i).dz=0;
rho_0 = 10;
f_tot=0;
eroflag = 0;
while sqrt((b(i).x-goalPoint(1))^2+(b(i).y-goalPoint(2))^2+(b(i).z-goalPoint(3))^2)>Thr
   
      % 计算引力
    if  sqrt((b(i).x-goalPoint(1))^2+(b(i).y-goalPoint(2))^2+(b(i).z-goalPoint(3))^2)>r_f
        f_att= [goalPoint(1)-b(i).x,goalPoint(2)-b(i).y,goalPoint(3)-b(i).z]*k_att;
    else
        f_att =[goalPoint(1)-b(i).x,goalPoint(2)-b(i).y,goalPoint(3)-b(i).z]*k_att*50/sqrt((b(i).x-goalPoint(1))^2+(b(i).y-goalPoint(2))^2+(b(i).z-goalPoint(3))^2);
    end
        
    % 初始化力向量
    f_rep = [0,0,0];
    f_rtex = [0,0,0];
    f_cutex = [0,0,0];
    f_cytex = [0,0,0];
    
        
      %贪婪检测
     %% 碰撞检测
        startp.x= b(i).x;
        startp.y =b(i).y;
        startp.z= b(i).z;
        endp.x = goalPoint(1);
        endp.y = goalPoint(2);
        endp.z = goalPoint(3);
        r = sqrt((startp.x-endp.x)^2+(startp.y-endp.y)^2+(startp.z-endp.z)^2);
        fai = atan2(endp.y-startp.y,endp.x-startp.x);
        theta = acos((endp.z-startp.z)/r); 
        cubeFlag = cubecheck(startp,cubeInfo,theta,fai,endp);%%长方体障碍物检测
        cylinderFlag = cylindercheck(startp,cylinderInfo,theta,fai,endp);%%圆柱体障碍物检测
        sphereFlag = spherecheck(startp,sphereInfo,theta,fai,endp);%%球体障碍物检测
         if ~(cubeFlag || cylinderFlag ||sphereFlag)
            b(i).dx = goalPoint(1)-b(i).x;
            b(i).dy = goalPoint(2)-b(i).y;
            b(i).dz = goalPoint(3)-b(i).z;
            b(i+1).x = goalPoint(1) ;
            b(i+1).y = goalPoint(2) ;
            b(i+1).z = goalPoint(3) ;
            b(i+1).dx = 0 ;
            b(i+1).dy = 0 ;
            b(i+1).dz = 0 ;
         line([b(i).x b(i+1).x], [b(i).y b(i+1).y;],[b(i).z b(i+1).z,], 'Linewidth', 2,'Color', 'k');
         break
         end
      %处理方形障碍物
        for j = 1:length(cubeInfo.axisX) 
            x1 = cubeInfo.axisX(j);
            y1 = cubeInfo.axisY(j);
            z1 = cubeInfo.axisZ(j);
            x2 = cubeInfo.axisX(j)+cubeInfo.lengthx(j);
            y2 = cubeInfo.axisY(j)+cubeInfo.lengthy(j);
            z2 = cubeInfo.axisZ(j)+cubeInfo.lengthz(j);
            
            % 检查当前位置是否在障碍物内部
            if b(i).x >= x1 && b(i).x <= x2 && b(i).y >= y1 && b(i).y <= y2 && b(i).z >= z1 && b(i).z <= z2
                disp('初始位置位于障碍物内');
                eroflag = 1;
                break;
            end
            
            % 计算与障碍物最近点的距离
            x_closest = b(i).x;
            y_closest = b(i).y ;
            z_closest = b(i).z ;
            if b(i).x < x1
                x_closest = x1;
            elseif b(i).x > x2
                x_closest = x2;
            end
            if b(i).y  < y1
                y_closest = y1;
            elseif b(i).y  > y2
                y_closest = y2;
            end
            if b(i).z  < z1
                z_closest = z1;
            elseif b(i).z  > z2
                z_closest = z2;
            end
            
            % 计算与最近点的距离
            dist_to_obstacle = norm([x_closest - b(i).x , y_closest - b(i).y ,z_closest - b(i).z]);
            
            % 如果距离小于影响范围，计算斥力和涡流力
            if dist_to_obstacle < rho_0
                % 计算斥力
                f_rep = f_rep + k_rep * (1/dist_to_obstacle - 1/rho_0) * 1/dist_to_obstacle^2 * ([b(i).x , b(i).y,b(i).z] - [x_closest, y_closest,z_closest]);
                
                %计算涡流力
                cube_center =[cubeInfo.axisX(j) + cubeInfo.lengthx(j)/2, cubeInfo.axisY(j) + cubeInfo.lengthy(j)/2,cubeInfo.axisZ(j)+cubeInfo.lengthz(j)/2];
               
                % 计算障碍物表面法线方向
                normal_direction = [b(i).x - x_closest, b(i).y - y_closest,b(i).z - z_closest];
                normal_direction = normal_direction / norm(normal_direction);
                            % 构建切平面坐标系
               if abs(normal_direction(1)) > 0.1
                base_vec = [0, 1, 0];
               else
                base_vec = [1, 0, 0];
               end
               
               %计算切向量
               % 计算切向量
               tangent1 = cross(normal_direction, base_vec);
               tangent1 = tangent1 / norm(tangent1);
               tangent2 = cross(normal_direction, tangent1);
               tangent2 = tangent2 / norm(tangent2);
                % 投影到切平面
               vec_to_current =  [b(i).x,b(i).y,b(i).z] - cube_center;
               vec_to_goal = [goalPoint(1),goalPoint(2),goalPoint(3)] - cube_center;
               proj_current = [dot(vec_to_current, tangent1), dot(vec_to_current, tangent2)];
               proj_goal = [dot(vec_to_goal, tangent1), dot(vec_to_goal, tangent2)];
                % 二维叉积判断绕行方向
                vec_robot = proj_current;
                vec_goal = proj_goal;
                cross_product = vec_robot(1)*vec_goal(2) - vec_robot(2)*vec_goal(1);
                 % 确定绕行方向
                if cross_product > 0
                vortex_direction_2d = [-vec_robot(2), vec_robot(1)]; % 逆时针
                else
                vortex_direction_2d = [vec_robot(2), -vec_robot(1)]; % 顺时针
                end
            
                % 归一化并转换回三维
               if norm(vortex_direction_2d) > 0
                vortex_direction_2d = vortex_direction_2d / norm(vortex_direction_2d);
                vortex_direction_3d = vortex_direction_2d(1) * tangent1 + vortex_direction_2d(2) * tangent2;
                
                % 计算涡流力
                vortex_magnitude = k_cutex * (1/dist_to_obstacle - 1/rho_0);
                f_cutex = vortex_magnitude * vortex_direction_3d;
               end
            end              
        end
        
        
            %处理柱形障碍物
          % 处理柱形障碍物
for j = 1:length(cylinderInfo.X)
    x1 = cylinderInfo.X(j);
    y1 = cylinderInfo.Y(j);
    z1 = cylinderInfo.Z(j);
    z2 = cylinderInfo.Z(j)+cylinderInfo.lengthZ(j);
    
    % 判断是否在障碍物内部
    if b(i).z>z1&&b(i).z<z2
        if sqrt((b(i).x-x1)^2+(b(i).y-y1)^2)<cylinderInfo.radius(j)
            disp('初始位置位于障碍物内');
              eroflag = 1;
            break;
        end
    end
    
    % 计算与障碍物最近点的距离  
    x_closest = b(i).x;
    y_closest = b(i).y;
    z_closest = b(i).z;
    
    if b(i).z < z1
        z_closest = z1;
    elseif b(i).z > z2
        z_closest = z2;
    end
    
    if sqrt((b(i).x-x1)^2+(b(i).y-y1)^2)>cylinderInfo.radius(j)
        temp = ([b(i).x-x1, b(i).y-y1]/max(norm([b(i).x-x1, b(i).y-y1]), 1e-10)*cylinderInfo.radius(j)+[x1, y1]);
        x_closest = temp(1);
        y_closest = temp(2);
    end
    
    % 计算与最近点的距离
    dist_to_obstacle = norm([x_closest - b(i).x, y_closest - b(i).y, z_closest - b(i).z]);
    % 调试信息
 
 
    if dist_to_obstacle > 0
        normal = [b(i).x-x_closest, b(i).y-y_closest, b(i).z-z_closest] / dist_to_obstacle;
    else
        normal = [1, 0, 0]; % 默认法向量
    end
    
    % 如果距离小于影响范围，计算斥力和涡流力
    if dist_to_obstacle < rho_0 && dist_to_obstacle > 0
        % 计算斥力
        f_rep = f_rep + k_rep * (1/dist_to_obstacle - 1/rho_0) * 1/dist_to_obstacle^2 * ([b(i).x, b(i).y, b(i).z] - [x_closest, y_closest, z_closest]);
        
        % 计算引力在切平面上的投影
        att_projection = f_att - dot(f_att, normal) * normal;
        
        % 如果引力与法线重合（投影很小），使用稳定的绕行方式
        if norm(att_projection) < 0.1
            % 稳定的绕行方式：基于当前位置选择固定方向
            dx = b(i).x - x1;
            dy = b(i).y - y1;
            if dx > 0
                vortex_direction = [-dy, dx, 0]; % 逆时针
            else
                vortex_direction = [dy, -dx, 0]; % 顺时针
            end
            
            if norm(vortex_direction) > 0
                vortex_direction = vortex_direction / norm(vortex_direction);
            else
                vortex_direction = [0, 1, 0]; % 默认方向
            end
        else
            % 使用引力在切平面的投影方向
            vortex_direction = att_projection / norm(att_projection);
        end
        
        % 计算涡流力大小
        vortex_magnitude = k_cytex * (1/dist_to_obstacle - 1/rho_0);
        f_cytex = f_cytex + vortex_magnitude * vortex_direction; % 修正变量名
    end
end

%处理球形障碍物
for j = 1:length(sphereInfo.X)
    x1 = sphereInfo.X(j);
    y1 = sphereInfo.Y(j);
    z1 = sphereInfo.Z(j);
    
    % 判断是否在障碍物内部
        if sqrt((b(i).x-x1)^2+(b(i).y-y1)^2+(b(i).z-z1)^2)<sphereInfo.radius(j)
            disp('初始位置位于障碍物内');
              eroflag = 1;
            break;
        end
    
    % 计算与障碍物最近点的距离  
    x_closest = b(i).x;
    y_closest = b(i).y;
    z_closest = b(i).z;
    
    if sqrt((b(i).x-x1)^2+(b(i).y-y1)^2+(b(i).z-z1)^2)>sphereInfo.radius(j)
        temps = ([b(i).x-x1, b(i).y-y1,b(i).z-z1]/max(norm([b(i).x-x1, b(i).y-y1,b(i).z-z1]), 1e-10)*cylinderInfo.radius(j)+[x1, y1,z1]);
        x_closest = temps(1);
        y_closest = temps(2);
        z_closest = temps(3);
    end
    
    % 计算与最近点的距离
    dist_to_obstacle = norm([x_closest - b(i).x, y_closest - b(i).y, z_closest - b(i).z]);
   
    if dist_to_obstacle > 0
        normals = [b(i).x-x_closest, b(i).y-y_closest, b(i).z-z_closest] / dist_to_obstacle;
    end
    
    % 如果距离小于影响范围，计算斥力和涡流力
    if dist_to_obstacle < rho_0 && dist_to_obstacle > 0
        % 计算斥力
        f_rep = f_rep + k_rep * (1/dist_to_obstacle - 1/rho_0) * 1/dist_to_obstacle^2 * ([b(i).x, b(i).y, b(i).z] - [x_closest, y_closest, z_closest]);
        
        % 计算引力在切平面上的投影
        att_projection = f_att - dot(f_att, normals) * normals;
        
        % 如果引力与法线重合（投影很小），使用稳定的绕行方式
        if norm(att_projection) < 0.1
            % 稳定的绕行方式：基于当前位置选择固定方向
            dx = b(i).x - x1;
            dy = b(i).y - y1;
            if dx > 0
                vortex_direction = [-dy, dx, 0]; % 逆时针
            else
                vortex_direction = [dy, -dx, 0]; % 顺时针
            end
            
            if norm(vortex_direction) > 0
                vortex_direction = vortex_direction / norm(vortex_direction);
            else
                vortex_direction = [0, 1, 0]; % 默认方向
            end
        else
            % 使用引力在切平面的投影方向
            vortex_direction = att_projection / norm(att_projection);
        end
        
        % 计算涡流力大小
        vortex_magnitude = k_rtex * (1/dist_to_obstacle - 1/rho_0);
        f_rtex = f_rtex + vortex_magnitude * vortex_direction; % 修正变量名
    end
end
      if eroflag
         break
     end
     % 合力计算 - 如果在狭窄通道中，添加额外的引导力
    f_tot = f_rep + f_att + f_rtex + f_cutex+f_cytex;
    if norm(f_rep)>norm(f_att)
        f_tot = f_rtex + f_cutex+f_cytex;
    end
    
    % 防止力过大
    force_magnitude = norm(f_tot);
    if force_magnitude > 5
        f_tot = f_tot / force_magnitude * 5;
    end
    
    % 速度更新
    if i<2||~(f_rep(1)==0&&f_rep(2)==0)
        b(i).dx = f_tot(1);
        b(i).dy = f_tot(2);
        b(i).dz = f_tot(3);
    else
        b(i).dx = b(i-1).dx + f_tot(1);
        b(i).dy = b(i-1).dy + f_tot(2);
        b(i).dz = b(i-1).dz + f_tot(3);
    end
    
    % 防止速度过大
    velocity_magnitude = norm([b(i).dx, b(i).dy, b(i).dz]);
    if velocity_magnitude > 3
        b(i).dx = b(i).dx / velocity_magnitude * 3;
        b(i).dy = b(i).dy / velocity_magnitude * 3;
         b(i).dz = b(i).dz / velocity_magnitude * 3;
    end
    if velocity_magnitude < 0.01
         f_tot =  f_rtex + f_cutex+f_cytex;
         b(i).dx = f_tot(1);
         b(i).dy = f_tot(2);
         b(i).dz = f_tot(3);
    end
  % 位置更新
    b(i+1).x = b(i).x + b(i).dx;
    b(i+1).y = b(i).y + b(i).dy;
    b(i+1).z = b(i).z + b(i).dz; 
    line([b(i).x b(i+1).x], [b(i).y b(i+1).y;],[b(i).z b(i+1).z,], 'Linewidth', 2,'Color', 'k');
    i = i+1;
end

   %%转入IRRT*优化
%% 三维RRT算法寻找路径点
%% 预先规划的节点纳入树中      
if i < maxSteps
    b = b(1:i+1);
end

%% 变量定义
gw=10;%网格宽度
Maxiter = 2000; %最大迭代次数
maxgridx=ceil(axisLWH(1)/gw);
maxgridy=ceil(axisLWH(2)/gw);
maxgridz=ceil(axisLWH(3)/gw);
%初始化树节点
mingrid.x=1;
mingrid.y=1;
mingrid.z=1;
iter = 0; %当前迭代次数
step = 10; %步长
o = 1;
Thr = 10 ;%范围阈值
rewire_Thr=8;
findPath = 0;
%% 初始化树
Tv(Maxiter + length(b)) = struct('x', 0, 'y', 0, 'z', 0, 'gridx', 0, 'gridy', 0, 'gridz', 0, 'pre', 0, 'cos', 0);
Tv(1).x = startPoint(1);
Tv(1).y = startPoint(2);
Tv(1).z = startPoint(3);
Tv(1).gridx = max(1,ceil(Tv(1).x/gw));
Tv(1).gridy = max(1,ceil(Tv(1).y/gw));
Tv(1).gridz = max(1,ceil(Tv(1).z/gw));
Tv(1).pre=0;
Tv(1).cos=0;
leng = [];
UpdateTime = 50;
cm = norm(goalPoint-startPoint)/2;
cz = (goalPoint+startPoint)/2;
gridin=cell(maxgridx,maxgridy,maxgridz);
gridin{Tv(1).gridx,Tv(1).gridy,Tv(1).gridz}(1)=1;
for o = 2:length(b)
    Tv(o).x = b(o).x;
    Tv(o).y = b(o).y;
    Tv(o).z = b(o).z;
    Tv(o).gridx = ceil(Tv(o).x /gw);
    Tv(o).gridy = ceil(Tv(o).z /gw);
    Tv(o).gridz = ceil(Tv(o).z /gw);
    Tv(o).pre=o-1;
    Tv(o).cos=Tv(o-1).cos+norm([b(o-1).dx,b(o-1).dy,b(o-1).dz]);
    gridin{Tv(o).gridx,Tv(o).gridy,Tv(o).gridz}(end+1)=o;
end
Goal_index = o;
 l=Tv(Goal_index).cos;
for iter = 1 : Maxiter
    if o>10000
        break
    end
cb = Tv(Goal_index).cos/2;  % 长轴半径
leng(iter) = 2*cb;          % 长轴长度
cs = sqrt(max(cb^2 - cm^2, 0)); % 短轴半径

vec = goalPoint - startPoint;  % 长轴方向向量
vec_length = norm(vec);        % 向量长度
if vec_length > 0
    vec_unit = vec / vec_length; % 单位向量
else
    vec_unit = [0, 0, 1]; % 默认Z轴方向
end

% 生成椭球坐标系的正交基
if abs(vec_unit(3)) > 0.9
    temp = [0, 1, 0];
else
    temp = [0, 0, 1];
end
ux = cross(vec_unit, temp);
ux = ux / norm(ux);
uy = cross(vec_unit, ux);
uy = uy / norm(uy);

% 均匀采样椭球内部
u = rand(1, 1)^(1/3);  % 保证体积均匀分布
v = rand(1, 1);
w = rand(1, 1);

% 在局部椭球坐标系中采样
theta = 2 * pi * v;
phi = pi * w;
r = u;

local_x = r * sin(phi) * cos(theta) * cs;
local_y = r * sin(phi) * sin(theta) * cs;
local_z = r * cos(phi) * cb;

% 转换到世界坐标系
center = (startPoint + goalPoint) / 2; % 椭球中心
world_point = center + local_x * ux + local_y * uy + local_z * vec_unit;

% 输出结果
randp.x = world_point(1);
randp.y = world_point(2);
randp.z = world_point(3);
    out = 1;
    %% 随机点在障碍物外
    for k  = 1:length(cubeInfo.axisX)
        if randp.x>= cubeInfo.axisX(k)&& randp.x<cubeInfo.axisX(k)+cubeInfo.lengthx(k)
            if randp.y>= cubeInfo.axisY(k)&& randp.y<cubeInfo.axisY(k)+cubeInfo.lengthy(k)
                if randp.z>= cubeInfo.axisZ(k)&& randp.z<cubeInfo.axisZ(k)+cubeInfo.lengthz(k)
                    out = 0; % invalid node
                    break
                end
            end
        end
    end
    for k  = 1:length(cylinderInfo.X)
        if sqrt((randp.x-cylinderInfo.X(k))^2+(randp.y-cylinderInfo.Y(k))^2)<cylinderInfo.radius(k)
            if randp.z>= cylinderInfo.Z(k)&& randp.z<cylinderInfo.Z(k)+cylinderInfo.lengthZ(k)
                out = 0; % invalid node
                break
            end
        end
    end
    for k  = 1:length(cylinderInfo.X)
        if sqrt((randp.x-sphereInfo.X(k))^2+(randp.y-sphereInfo.Y(k))^2+(randp.y-sphereInfo.Z(k))^2)<sphereInfo.radius(k)           
                out = 0; % invalid node
                break
        end
    end
    if out==0
        continue;
    end
    %% 寻找最近的点
    tgrid.x=ceil(randp.x/gw);
    tgrid.y=ceil(randp.y/gw);
    tgrid.z=ceil(randp.z/gw);
    x_near=[];
    c=1;
    while isempty(x_near)
        for i=max(tgrid.x-c,1):min(tgrid.x+c,maxgridx)
            for j=max(tgrid.y-c,1):min(tgrid.y+c,maxgridy)
                for m=max(tgrid.z-c,1):min(tgrid.z+c,maxgridz)
                    for k=1:length(gridin{i,j,m})
                        x_near(end+1)=gridin{i,j,m}(k);
                    end
                end
            end
        end
        c=c+1;
    end
    
    distance = [];
    k3 = 1;
    while k3 <= length(x_near) % should not use size() function
        dx =  randp.x - Tv(x_near(k3)).x;
        dy =  randp.y - Tv(x_near(k3)).y;
        dz =  randp.z - Tv(x_near(k3)).z;
        distance(k3) = sqrt(dx^2 + dy^2+dz^2);
        k3 = k3+1; 
    end
    
    [~,min_idx] = min(distance);
    r = distance(min_idx);
    min_idx = x_near(min_idx);
    nearp.x= Tv(min_idx).x;
    nearp.y= Tv(min_idx).y;
    nearp.z= Tv(min_idx).z;
    nearp.cos=Tv(min_idx).cos;
   %% 拓展得到新节点 
  
   fai = atan2(randp.y-nearp.y,randp.x-nearp.x);
   theta = acos((randp.z-nearp.z)/r);
   newp.x =  real(nearp.x+step*sin(theta)*cos(fai));
   newp.y =  real(nearp.y+step*sin(theta)*sin(fai));
   newp.z =  real(nearp.z+step*cos(theta));
   newp.cos = nearp.cos+step;
   if newp.x<axisStart(1)||newp.x>axisLWH(1)||newp.y<axisStart(2)||newp.y>axisLWH(2)||newp.z<axisStart(3)||newp.z>axisLWH(3)
       continue;
   end
   %% 碰撞检测
   cubeFlag = cubecheck(nearp,cubeInfo,theta,fai,newp);%%长方体障碍物检测
   cylinderFlag = cylindercheck(nearp,cylinderInfo,theta,fai,newp);%%圆柱体障碍物检测
   sphereFlag = spherecheck(nearp,sphereInfo,theta,fai,newp);%%球体障碍物检测
   if cubeFlag || cylinderFlag ||sphereFlag
       continue;
   else
   %% 重连父节点,设置最近节点为临时父节点
   tempparent.x= nearp.x;
   tempparent.y= nearp.y;
   tempparent.z= nearp.z;
   temppre = min_idx;
   tempcos=newp.cos;
   
   %重连父节点
    newgridx=ceil(newp.x/gw);
    newgridy=ceil(newp.y/gw);
    newgridz=ceil(newp.z/gw);
    x_newnear=[];
    for i=max(newgridx-1,1):min(newgridx+1,maxgridx)
        for j=max(newgridy-1,1):min(newgridy+1,maxgridy)
            for m=max(newgridz-1,1):min(newgridz+1,maxgridz)
                for k=1:length(gridin{i,j,m})
                    x_newnear(end+1)=gridin{i,j,m}(k);
                end
            end
        end
    end
    
   %% 寻找范围内的点 
    rewirep = [];
    rewirecount = [];
    co= [];
    for k3 = 1:length(x_newnear) % should not use size() function
        checkp.x= Tv(x_newnear(k3)).x;
        checkp.y= Tv(x_newnear(k3)).y;
        checkp.z= Tv(x_newnear(k3)).z;
        checkp.cos = Tv(x_newnear(k3)).cos;
        dx =  newp.x - Tv(x_newnear(k3)).x;
        dy =  newp.y - Tv(x_newnear(k3)).y;
        dz =  newp.z - Tv(x_newnear(k3)).z;
        co(k3) = sqrt(dx^2+dy^2+dz^2); 
        %% 碰撞检测
         cubeFlag = cubecheck(nearp,cubeInfo,theta,fai,checkp);%%长方体障碍物检测
         cylinderFlag = cylindercheck(nearp,cylinderInfo,theta,fai,checkp);%%圆柱体障碍物检测
         sphereFlag = spherecheck(nearp,sphereInfo,theta,fai,checkp);%%球体障碍物检测
         if cubeFlag || cylinderFlag ||sphereFlag
            continue;
         else
             rewirep(end+1)=x_newnear(k3);
             rewirecount(end+1) = k3;
             if checkp.cos+co(k3)<tempcos
                  tempcos=checkp.cos+co(k3);
                  tempparent.x= checkp.x;
                  tempparent.y= checkp.y;
                  tempparent.z= checkp.z;
                  temppre = x_newnear(k3);
             end
         end
        
    end

       %% 纳入新节点
       o=o+1;
       Tv(o).x=newp.x;
       Tv(o).y=newp.y;
       Tv(o).z=newp.z;
       Tv(o).pre=temppre;
       Tv(o).cos=tempcos;
       Tv(o).gridx = ceil(Tv(o).x /gw);
       Tv(o).gridy = ceil(Tv(o).y /gw);
       Tv(o).gridz = ceil(Tv(o).z /gw);
       gridin{ Tv(o).gridx ,Tv(o).gridy,Tv(o).gridz}(end+1)=o;
       line([Tv(temppre).x  newp.x],[Tv(temppre).y  newp.y],[Tv(temppre).z  newp.z],'LineWidth',1,'Color','b');
       pause(0.01);
       %% 重布线
        for k4=1:length(rewirep)
            if co(rewirecount(k4))+tempcos<Tv(rewirep(k4)).cos
                Tv(rewirep(k4)).cos = co(rewirecount(k4))+tempcos;
                Tv(rewirep(k4)).pre = o-1;
            end
        end 
       if iter == Maxiter
           Path=[];
           disp('路径规划失败');
           return;
       end
   end
  
    %Step 8:检查是否到达目标点附近 
    vector = [newp.x,newp.y,newp.z]-goalPoint;
    disToGoal = norm(vector);
    update_count = update_count + 1;
    if(update_count == UpdateTime)
        update_count = 0;
        j = 2;
        path.pos(1).x = goalPoint(1);
        path.pos(1).y = goalPoint(2);
        path.pos(1).z = goalPoint(3);
        pathIndex = Tv(Goal_index).pre;
        while 1
            path.pos(j).x = Tv(pathIndex).x;
            path.pos(j).y = Tv(pathIndex).y;
            path.pos(j).z = Tv(pathIndex).z;
            pathIndex = Tv(pathIndex).pre;    % 沿终点回溯到起点
            if pathIndex == 0
                break
            end
            j=j+1;
        end
    end
	%pause(DelayTime); %暂停DelayTime s,使得RRT*扩展过程容易观察
    if   Goal_index
    leng(iter)= Tv(Goal_index).cos;
    end
 
 
end
j = 2;
        path.pos(1).x = goalPoint(1);
        path.pos(1).y = goalPoint(2);
        path.pos(1).z = goalPoint(3);
        pathIndex = Tv(Goal_index).pre;
        while 1
            path.pos(j).x = Tv(pathIndex).x;
            path.pos(j).y = Tv(pathIndex).y;
            path.pos(j).z = Tv(pathIndex).z;
            pathIndex = Tv(pathIndex).pre;    % 沿终点回溯到起点
            if pathIndex == 0
                break
            end
            j=j+1;
        end
for i = 1:length(path)-1
    line([path.pos(i).x path.pos(i+1).x],[path.pos(i).y path.pos(i+1).y],[path.pos(i).z path.pos(i+1).z],'LineWidth',1,'Color','r');
end
% %% 回溯父节点
% index = Tv(end).pre;
% o = 1;
% while Tv(index).pre~=0
%     Path(o,1) = Tv(index).x;
%     Path(o,2) = Tv(index).y;
%     Path(o,3) = Tv(index).z;
%     index = Tv(index).pre;
%     o =o+1;
% end
%  Path(o,1) = startPoint(1);
%  Path(o,2) = startPoint(2);
%  Path(o,3) = startPoint(3);
%  
%  Path = flipud(Path);
%  Path(o+1,1) = goalPoint(1);
%  Path(o+1,2) = goalPoint(2);
%  Path(o+1,3) = goalPoint(3);
% 
% 
% if ~isempty(Path)
%  for k2=1:size(Path,1)-1
%     line([Path(k2,1) Path(k2+1,1)],[Path(k2,2) Path(k2+1,2)],[Path(k2,3) Path(k2+1,3)],'LineWidth',1,'Color','r');
%  end
%  totalPath = [totalPath;Path];
% end
% toc
