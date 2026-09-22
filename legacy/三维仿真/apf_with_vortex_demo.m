%%清空变量
clear all
clc

%%定义变量
axisStart = [0 0 0];
axisLWH=[150 150 150];

%%定义障碍物
cubeInfo.exist=0;
cylinderInfo.exist=0;
sphereInfo.exist=0;


%%定义初始点
pathPoint= [0 0 0;
            150 150 90];  %路径点
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
pellucidity = 0.6 ;%透明度
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
b(i).x=startPoint(1) ;
b(i).y=startPoint(2) ;
b(i).z=startPoint(3) ;
b(i).dx=0;
b(i).dy=0;
b(i).dz=0;
rho_0 = 10;
f_tot=0;

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
        cylinderFlag = cylindercheck(startp,cylinderInfo,theta,fai,endp);%%圆柱体障22碍物检测
        sphereFlag = spherecheck(startp,sphereInfo,theta,fai,endp);%%球体障碍物检测
         if ~(cubeFlag || cylinderFlag ||sphereFlag)
             disp('可以直通目标点'); 
            b(i).dx = goalPoint(1)-b(i).x;
            b(i).dy = goalPoint(2)-b(i).y;
            b(i).dz = goalPoint(3)-b(i).z;
            b(i+1).x = goalPoint(1) ;
            b(i+1).y = goalPoint(2) ;
            b(i+1).z = goalPoint(3) ;
            b(i+1).dx = 0 ;
            b(i+1).dy = 0 ;
            b(i+1).dz = 0 ;
        line([b(i).x b(i+1).x], [b(i).y b(i+1).y;],[b(i).z b(i+1).z,], 'Linewidth', 2,'Color', 'b');
        break
         end
    
   
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
    fprintf('柱形障碍物 %d: 机器人位置 (%.2f, %.2f, %.2f), 障碍物中心 (%.2f, %.2f, %.2f) 到 %.2f, 半径 %.2f\n', ...
        j, b(i).x, b(i).y, b(i).z, x1, y1, z1, z2, cylinderInfo.radius(j));
    fprintf('最近点 (%.2f, %.2f, %.2f), 距离 %.2f, 影响范围 %.2f\n', ...
        x_closest, y_closest, z_closest, dist_to_obstacle, rho_0);
 
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
    line([b(i).x b(i+1).x], [b(i).y b(i+1).y;],[b(i).z b(i+1).z,], 'Linewidth', 2,'Color', 'b');
    i = i+1;
    
    % 绘制路径 - 狭窄通道用红色标记


    end


