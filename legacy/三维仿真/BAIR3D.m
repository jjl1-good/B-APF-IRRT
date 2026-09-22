function [a,t] = BAIR3D()
tic 

% 性能优化设置
feature('accel','on');
warning('off', 'MATLAB:colon:nonIntegerIndex');

% 定义变量
axisStart = [0 0 0];
axisLWH=[200 200 150];

% 定义障碍物
cubeInfo.exist=0;
cylinderInfo.exist=0;
sphereInfo.exist=0;

% 定义初始点
pathPoint= [26 26 1; 175 175 150];
update_count = 0;      
cubeInfo = createCube(cubeInfo);
cylinderInfo = createcylinder(cylinderInfo);
sphereInfo =createSphere(sphereInfo);
influencerange = 10;

%% 创建可视化窗口 - 注释掉
% figure(1);
% set(gcf, 'Position', [100, 100, 1400, 800]);
% 
% % 主路径规划视图
% subplot(2,3,[1,2,4,5]);
% hold on;
% grid on;
% axis equal;
% axis([0 200 0 200 0 150]);
% view(45, 30);
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('三维路径规划 - BUG-APF + IRRT*', 'FontSize', 12, 'FontWeight', 'bold');
% 
% %绘制障碍物
% colorMatCube = [1 0 0]; 
% colorMatCylinder = [0 1 0];
% colorMatSphere = [0 0 1];
% pellucidity = 0.3;
% 
% drawCube(cubeInfo,colorMatCube,pellucidity);
% drawcylinder(cylinderInfo,colorMatCylinder,pellucidity);
% drawsphere(sphereInfo,colorMatSphere,pellucidity);
% 
% %绘制起点和终点
startPoint = pathPoint(1,:);
goalPoint = pathPoint(2,:);
% scatter3(startPoint(1), startPoint(2), startPoint(3), 120, 'g', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
% text(startPoint(1), startPoint(2), startPoint(3), '  起点', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'g');
% 
% scatter3(goalPoint(1), goalPoint(2), goalPoint(3), 120, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
% text(goalPoint(1), goalPoint(2), goalPoint(3), '  终点', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');
% 
% % 创建图例
% legend_items = [];
% legend_items(1) = scatter3(NaN, NaN, NaN, 100, 'g', 'filled', 'DisplayName', '起点');
% legend_items(2) = scatter3(NaN, NaN, NaN, 100, 'r', 'filled', 'DisplayName', '终点');
% legend_items(3) = patch(NaN, NaN, 'r', 'FaceAlpha', 0.3, 'DisplayName', '立方体障碍物');
% legend_items(4) = patch(NaN, NaN, 'g', 'FaceAlpha', 0.3, 'DisplayName', '圆柱障碍物');
% legend_items(5) = patch(NaN, NaN, 'b', 'FaceAlpha', 0.3, 'DisplayName', '球体障碍物');
% legend(legend_items, 'Location', 'northeast');
% 
% drawnow;

totalPath=[];
Thr = 2;
k_att=0.001; k_rtex = 0.002; k_cutex = 0.002; k_cytex = 0.002; k_rep=0.9; r_f=50;

%% 进度显示子图 - 注释掉
% subplot(2,3,3);
% progress_text = text(0.1, 0.5, '准备开始 BUG-APF 路径规划...', 'FontSize', 12, 'FontWeight', 'bold');
% axis off;
% title('算法状态', 'FontSize', 12, 'FontWeight', 'bold');
% 
% %% 统计信息子图 - 注释掉
% subplot(2,3,6);
% stats_text = text(0.1, 0.7, '', 'FontSize', 10);
% axis off;
% title('运行统计', 'FontSize', 12, 'FontWeight', 'bold');

%% 改进的BUG-APF寻路 - 使用完整的碰撞检测
maxSteps = 10000; % 增加最大步数
b_x = zeros(maxSteps+1, 1);
b_y = zeros(maxSteps+1, 1);
b_z = zeros(maxSteps+1, 1);
b_dx = zeros(maxSteps+1, 1);
b_dy = zeros(maxSteps+1, 1);
b_dz = zeros(maxSteps+1, 1);

b_x(1) = startPoint(1);
b_y(1) = startPoint(2);
b_z(1) = startPoint(3);
rho_0 = 15; % 增加影响范围
eroflag = 0;

% 改进的力常数设置
k_att = 0.005;  % 增加引力常数
k_rep = 1.5;    % 调整斥力常数
k_rtex = 0.005; % 增加涡流力常数
k_cutex = 0.005;
k_cytex = 0.005;

% 可视化设置 - 注释掉
% subplot(2,3,[1,2,4,5]);
% bug_path_plot = plot3(NaN, NaN, NaN, 'k-', 'LineWidth', 3, 'DisplayName', 'BUG-APF路径');
% current_pos_plot = scatter3(NaN, NaN, NaN, 80, 'm', 'filled', 'DisplayName', '当前位置');
% force_plot = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 'r', 'LineWidth', 2, 'MaxHeadSize', 1, 'DisplayName', '合力方向');
% legend('show');
% 
% % 更新进度显示
% subplot(2,3,3);
% set(progress_text, 'String', '正在进行 BUG-APF 路径规划...');

i = 1;
path_length = 0;
stuck_counter = 0;
last_position = [b_x(1), b_y(1), b_z(1)];
while sqrt((b_x(i)-goalPoint(1))^2 + (b_y(i)-goalPoint(2))^2 + (b_z(i)-goalPoint(3))^2) > Thr && i < maxSteps
   
    % 计算引力 - 改进的引力函数
    dist_to_goal = sqrt((b_x(i)-goalPoint(1))^2 + (b_y(i)-goalPoint(2))^2 + (b_z(i)-goalPoint(3))^2);
    if dist_to_goal > r_f
        f_att = [goalPoint(1)-b_x(i), goalPoint(2)-b_y(i), goalPoint(3)-b_z(i)] * k_att;
    else
        % 在接近目标时增加引力强度
        f_att = [goalPoint(1)-b_x(i), goalPoint(2)-b_y(i), goalPoint(3)-b_z(i)] * k_att * 100 / dist_to_goal;
    end
    
    % 初始化力向量
    f_rep = [0,0,0];
    f_rtex = [0,0,0];
    f_cutex = [0,0,0];
    f_cytex = [0,0,0];
    
    % 贪婪检测 - 使用完整的碰撞检测
    startp.x = b_x(i); startp.y = b_y(i); startp.z = b_z(i);
    endp.x = goalPoint(1); endp.y = goalPoint(2); endp.z = goalPoint(3);
    r = sqrt((startp.x-endp.x)^2 + (startp.y-endp.y)^2 + (startp.z-endp.z)^2);
    
    if r > 0
        fai = atan2(endp.y-startp.y, endp.x-startp.x);
        cos_theta = (endp.z-startp.z)/r;
        cos_theta = max(min(cos_theta, 1), -1);
        theta = acos(cos_theta);
    else
        fai = 0; theta = 0;
    end
    
    cubeFlag = cubecheck(startp,cubeInfo,theta,fai,endp);
    cylinderFlag = cylindercheck(startp,cylinderInfo,theta,fai,endp);
    sphereFlag = spherecheck(startp,sphereInfo,theta,fai,endp);
    
    if ~(cubeFlag || cylinderFlag || sphereFlag)
        % 直接路径无碰撞，直接前往目标
        b_dx(i) = goalPoint(1)-b_x(i);
        b_dy(i) = goalPoint(2)-b_y(i);
        b_dz(i) = goalPoint(3)-b_z(i);
        b_x(i+1) = goalPoint(1);
        b_y(i+1) = goalPoint(2);
        b_z(i+1) = goalPoint(3); 
        b_dx(i+1) =0;
        b_dy(i+1) =0;
        b_dz(i+1) =0;
        
        % 更新可视化 - 直接到达目标 - 注释掉
        % set(bug_path_plot, 'XData', b_x(1:i+1), 'YData', b_y(1:i+1), 'ZData', b_z(1:i+1));
        % set(current_pos_plot, 'XData', b_x(i+1), 'YData', b_y(i+1), 'ZData', b_z(i+1));
        % set(force_plot, 'XData', b_x(i), 'YData', b_y(i), 'ZData', b_z(i), ...
        %     'UData', b_dx(i)*5, 'VData', b_dy(i)*5, 'WData', b_dz(i)*5);
        % drawnow;
        
%         fprintf('BUG-APF通过贪婪检测直接到达目标！\n');
        break
    end
     
    % 处理方形障碍物 - 使用完整的物理模型
    for j = 1:length(cubeInfo.axisX) 
        x1 = cubeInfo.axisX(j);
        y1 = cubeInfo.axisY(j);
        z1 = cubeInfo.axisZ(j);
        x2 = cubeInfo.axisX(j)+cubeInfo.lengthx(j);
        y2 = cubeInfo.axisY(j)+cubeInfo.lengthy(j);
        z2 = cubeInfo.axisZ(j)+cubeInfo.lengthz(j);
        
        % 检查当前位置是否在障碍物内部
        if b_x(i) >= x1 && b_x(i) <= x2 && b_y(i) >= y1 && b_y(i) <= y2 && b_z(i) >= z1 && b_z(i) <= z2
            fprintf('初始位置位于障碍物内\n');
            eroflag = 1;
            break;
        end
        
        % 计算与障碍物最近点的距离
        x_closest = b_x(i);
        y_closest = b_y(i);
        z_closest = b_z(i);
        if b_x(i) < x1
            x_closest = x1;
        elseif b_x(i) > x2
            x_closest = x2;
        end
        if b_y(i)  < y1
            y_closest = y1;
        elseif b_y(i)  > y2
            y_closest = y2;
        end
        if b_z(i)  < z1
            z_closest = z1;
        elseif b_z(i)  > z2
            z_closest = z2;
        end
        
        % 计算与最近点的距离
        dist_to_obstacle = norm([x_closest - b_x(i) , y_closest - b_y(i) ,z_closest - b_z(i)]);
        
        % 如果距离小于影响范围，计算斥力和涡流力
        if dist_to_obstacle < rho_0 && dist_to_obstacle > 0
            % 计算斥力
            f_rep = f_rep + k_rep * (1/dist_to_obstacle - 1/rho_0) * 1/dist_to_obstacle^2 * ([b_x(i) , b_y(i),b_z(i)] - [x_closest, y_closest,z_closest]);
            
            %计算涡流力
            cube_center =[cubeInfo.axisX(j) + cubeInfo.lengthx(j)/2, cubeInfo.axisY(j) + cubeInfo.lengthy(j)/2,cubeInfo.axisZ(j)+cubeInfo.lengthz(j)/2];
           
            % 计算障碍物表面法线方向
            normal_direction = [b_x(i) - x_closest, b_y(i) - y_closest,b_z(i) - z_closest];
            if norm(normal_direction) > 0
                normal_direction = normal_direction / norm(normal_direction);
            else
                normal_direction = [1, 0, 0]; % 默认方向
            end
            
            % 构建切平面坐标系
           if abs(normal_direction(1)) > 0.1
            base_vec = [0, 1, 0];
           else
            base_vec = [1, 0, 0];
           end
           
           %计算切向量
           tangent1 = cross(normal_direction, base_vec);
           if norm(tangent1) > 0
               tangent1 = tangent1 / norm(tangent1);
           else
               tangent1 = [0, 1, 0]; % 默认切向量
           end
           tangent2 = cross(normal_direction, tangent1);
           if norm(tangent2) > 0
               tangent2 = tangent2 / norm(tangent2);
           else
               tangent2 = [0, 0, 1]; % 默认切向量
           end
           
            % 投影到切平面
           vec_to_current =  [b_x(i),b_y(i),b_z(i)] - cube_center;
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
            f_cutex = f_cutex + vortex_magnitude * vortex_direction_3d;
           end
        end              
    end
    
    % 处理柱形障碍物 - 使用完整的物理模型
    for j = 1:length(cylinderInfo.X)
        x1 = cylinderInfo.X(j);
        y1 = cylinderInfo.Y(j);
        z1 = cylinderInfo.Z(j);
        z2 = cylinderInfo.Z(j)+cylinderInfo.lengthZ(j);
        
        % 判断是否在障碍物内部
        if b_z(i)>z1&&b_z(i)<z2
            if sqrt((b_x(i)-x1)^2+(b_y(i)-y1)^2)<cylinderInfo.radius(j)
                fprintf('初始位置位于障碍物内\n');
                eroflag = 1;
                break;
            end
        end
        
        % 计算与障碍物最近点的距离  
        x_closest = b_x(i);
        y_closest = b_y(i);
        z_closest = b_z(i);
        
        if b_z(i) < z1
            z_closest = z1;
        elseif b_z(i) > z2
            z_closest = z2;
        end
        
        if sqrt((b_x(i)-x1)^2+(b_y(i)-y1)^2)>cylinderInfo.radius(j)
            temp_vec = [b_x(i)-x1, b_y(i)-y1];
            temp_norm = norm(temp_vec);
            if temp_norm > 0
                temp = (temp_vec/temp_norm)*cylinderInfo.radius(j) + [x1, y1];
                x_closest = temp(1);
                y_closest = temp(2);
            else
                x_closest = x1 + cylinderInfo.radius(j);
                y_closest = y1;
            end
        end
        
        % 计算与最近点的距离
        dist_to_obstacle = norm([x_closest - b_x(i), y_closest - b_y(i), z_closest - b_z(i)]);
       
        if dist_to_obstacle > 0
            normal = [b_x(i)-x_closest, b_y(i)-y_closest, b_z(i)-z_closest] / dist_to_obstacle;
        else
            normal = [1, 0, 0]; % 默认法向量
        end
        
        % 如果距离小于影响范围，计算斥力和涡流力
        if dist_to_obstacle < rho_0 && dist_to_obstacle > 0
            % 计算斥力
            f_rep = f_rep + k_rep * (1/dist_to_obstacle - 1/rho_0) * 1/dist_to_obstacle^2 * ([b_x(i), b_y(i), b_z(i)] - [x_closest, y_closest, z_closest]);
            
            % 计算引力在切平面上的投影
            att_projection = f_att - dot(f_att, normal) * normal;
            
            % 如果引力与法线重合（投影很小），使用稳定的绕行方式
            if norm(att_projection) < 0.1
                % 稳定的绕行方式：基于当前位置选择固定方向
                dx = b_x(i) - x1;
                dy = b_y(i) - y1;
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
            f_cytex = f_cytex + vortex_magnitude * vortex_direction;
        end
    end

    % 处理球形障碍物 - 使用完整的物理模型
    for j = 1:length(sphereInfo.X)
        x1 = sphereInfo.X(j);
        y1 = sphereInfo.Y(j);
        z1 = sphereInfo.Z(j);
        
        % 判断是否在障碍物内部
        if sqrt((b_x(i)-x1)^2+(b_y(i)-y1)^2+(b_z(i)-z1)^2)<sphereInfo.radius(j)
            fprintf('初始位置位于障碍物内\n');
            eroflag = 1;
            break;
        end
        
        % 计算与障碍物最近点的距离  
        x_closest = b_x(i);
        y_closest = b_y(i);
        z_closest = b_z(i);
        
        if sqrt((b_x(i)-x1)^2+(b_y(i)-y1)^2+(b_z(i)-z1)^2)>sphereInfo.radius(j)
            temp_vec = [b_x(i)-x1, b_y(i)-y1, b_z(i)-z1];
            temp_norm = norm(temp_vec);
            if temp_norm > 0
                temp = (temp_vec/temp_norm)*sphereInfo.radius(j) + [x1, y1, z1];
                x_closest = temp(1);
                y_closest = temp(2);
                z_closest = temp(3);
            else
                x_closest = x1 + sphereInfo.radius(j);
                y_closest = y1;
                z_closest = z1;
            end
        end
        
        % 计算与最近点的距离
        dist_to_obstacle = norm([x_closest - b_x(i), y_closest - b_y(i), z_closest - b_z(i)]);
       
        if dist_to_obstacle > 0
            normals = [b_x(i)-x_closest, b_y(i)-y_closest, b_z(i)-z_closest] / dist_to_obstacle;
        else
            normals = [1, 0, 0]; % 默认法向量
        end
        
        % 如果距离小于影响范围，计算斥力和涡流力
        if dist_to_obstacle < rho_0 && dist_to_obstacle > 0
            % 计算斥力
            f_rep = f_rep + k_rep * (1/dist_to_obstacle - 1/rho_0) * 1/dist_to_obstacle^2 * ([b_x(i), b_y(i), b_z(i)] - [x_closest, y_closest, z_closest]);
            
            % 计算引力在切平面上的投影
            att_projection = f_att - dot(f_att, normals) * normals;
            
            % 如果引力与法线重合（投影很小），使用稳定的绕行方式
            if norm(att_projection) < 0.1
                % 稳定的绕行方式：基于当前位置选择固定方向
                dx = b_x(i) - x1;
                dy = b_y(i) - y1;
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
            f_rtex = f_rtex + vortex_magnitude * vortex_direction;
        end
    end
    
    if eroflag
        break
    end
     
    % 改进的合力计算
    f_tot = f_rep + f_att + f_rtex + f_cutex + f_cytex;
    if norm(f_rep) > norm(f_att)
        f_tot = f_rtex + f_cutex + f_cytex;
    end
    
    % 防止在局部最小值停滞
    current_position = [b_x(i), b_y(i), b_z(i)];
    if norm(current_position - last_position) < 0.1
        stuck_counter = stuck_counter + 1;
        if stuck_counter > 50
            % 添加随机扰动跳出局部最小值
            random_perturbation = 0.5 * (rand(1,3) - 0.5);
            f_tot = f_tot + random_perturbation;
            fprintf('检测到局部最小值，添加随机扰动\n');
            stuck_counter = 0;
        end
    else
        stuck_counter = 0;
        last_position = current_position;
    end
    
    % 防止力过大
    force_magnitude = norm(f_tot);
    if force_magnitude > 8
        f_tot = f_tot / force_magnitude * 8;
    elseif force_magnitude < 0.1
        % 如果力太小，增加目标导向性
        f_tot = f_tot + 0.5 * f_att / norm(f_att);
    end
    
    % 速度更新
    b_dx(i) = f_tot(1);
    b_dy(i) = f_tot(2);
    b_dz(i) = f_tot(3);
    
    % 防止速度过大
    velocity_magnitude = norm([b_dx(i), b_dy(i), b_dz(i)]);
    if velocity_magnitude > 4
        b_dx(i) = b_dx(i) / velocity_magnitude * 4;
        b_dy(i) = b_dy(i) / velocity_magnitude * 4;
        b_dz(i) = b_dz(i) / velocity_magnitude * 4;
    end
    
    % 位置更新
    b_x(i+1) = b_x(i) + b_dx(i);
    b_y(i+1) = b_y(i) + b_dy(i);
    b_z(i+1) = b_z(i) + b_dz(i);
    
    % 计算路径长度
    if i > 1
        path_length = path_length + norm([b_x(i)-b_x(i-1), b_y(i)-b_y(i-1), b_z(i)-b_z(i-1)]);
    end
    
    % 更新可视化 - 每20步更新一次以提高性能 - 注释掉
    % if mod(i, 20) == 0 || i < 10 || i > maxSteps-100
    %     set(bug_path_plot, 'XData', b_x(1:i+1), 'YData', b_y(1:i+1), 'ZData', b_z(1:i+1));
    %     set(current_pos_plot, 'XData', b_x(i+1), 'YData', b_y(i+1), 'ZData', b_z(i+1));
    %     set(force_plot, 'XData', b_x(i), 'YData', b_y(i), 'ZData', b_z(i), ...
    %         'UData', b_dx(i)*5, 'VData', b_dy(i)*5, 'WData', b_dz(i)*5);
    %     
    %     % 更新进度显示
    %     subplot(2,3,3);
    %     progress_str = sprintf('BUG-APF进度: %d/%d 步\n距离目标: %.2f\n路径长度: %.2f', ...
    %         i, maxSteps, dist_to_goal, path_length);
    %     set(progress_text, 'String', progress_str);
    %     
    %     % 更新统计信息
    %     subplot(2,3,6);
    %     stats_str = sprintf('已走步数: %d\n当前位置: [%.1f, %.1f, %.1f]\n距离目标: %.2f\n路径长度: %.2f', ...
    %         i, b_x(i), b_y(i), b_z(i), dist_to_goal, path_length);
    %     set(stats_text, 'String', stats_str);
    %     
    %     drawnow limitrate;
    % end
    
    i = i+1;
    
    % 如果接近目标，提前退出
    if dist_to_goal < Thr * 3
        break;
    end
end

% 裁剪数组
b_x = b_x(1:i+1);
b_y = b_y(1:i+1);
b_z = b_z(1:i+1);
b_dx = b_dx(1:i+1);
b_dy = b_dy(1:i+1);
b_dz = b_dz(1:i+1);

% 最终更新BUG-APF路径 - 注释掉
% set(bug_path_plot, 'XData', b_x, 'YData', b_y, 'ZData', b_z);
% set(current_pos_plot, 'XData', b_x(end), 'YData', b_y(end), 'ZData', b_z(end));

% 计算最终路径长度
bug_apf_length = 0;
for k = 2:length(b_x)
    bug_apf_length = bug_apf_length + norm([b_x(k)-b_x(k-1), b_y(k)-b_y(k-1), b_z(k)-b_z(k-1)]);
end

% fprintf('BUG-APF完成，路径点数量: %d，路径长度: %.2f\n', length(b_x), bug_apf_length);





% 更新进度显示 - 注释掉
% subplot(2,3,3);
% set(progress_text, 'String', sprintf('BUG-APF完成!\n路径点: %d\n路径长度: %.2f\n开始IRRT*优化...', length(b_x), bug_apf_length));

%% 改进的IRRT*算法 - 删除网格化，使用高效线性搜索
% fprintf('开始IRRT*优化...\n');

% IRRT*参数
Maxiter = 50000; % 最大迭代次数
step = 6; % 步长
Thr = 8; % 目标阈值
rewire_Thr = 15; % 重布线阈值

% 使用数组替代结构体
maxNodes = Maxiter + 1000;
Tv_x = zeros(maxNodes, 1);
Tv_y = zeros(maxNodes, 1);
Tv_z = zeros(maxNodes, 1);
Tv_pre = zeros(maxNodes, 1);
Tv_cos = zeros(maxNodes, 1);

% 初始化树
Tv_x(1) = startPoint(1); 
Tv_y(1) = startPoint(2); 
Tv_z(1) = startPoint(3);
Tv_pre(1) = 0; 
Tv_cos(1) = 0;

node_count = 1;
best_length = bug_apf_length;
Goal_index = 0;
findpath = 0;

% 统计信息
rewire_count = 0;
parent_select_count = 0;

% 添加IRRT*可视化 - 注释掉
% subplot(2,3,[1,2,4,5]);
% rrt_tree_plot = plot3(NaN, NaN, NaN, 'b-', 'LineWidth', 0.5, 'DisplayName', 'RRT*树', 'Color', [0.7 0.7 1]);
% rrt_nodes_plot = scatter3(NaN, NaN, NaN, 20, 'c', 'filled', 'MarkerFaceAlpha', 0.5, 'DisplayName', 'RRT*节点');
% final_path_plot = plot3(NaN, NaN, NaN, 'r-', 'LineWidth', 4, 'DisplayName', '最终路径');
% legend('show');
% 
% % 进度显示
% progress_interval = 500;

% fprintf('初始路径长度: %.2f\n', bug_apf_length);
lb = length(b_x);
b1 = 2;
for iter = 1:Maxiter
    if node_count >12000
        break
    end
    if node_count<6000
        rewire_Thr = 30;
    else
        rewire_Thr = 15;
    end
    % 进度显示 - 注释掉
    % if mod(iter, progress_interval) == 0
    %     fprintf('进度: %d/%d (%.1f%%) 当前节点数: %d，最佳路径长度: %.2f\n', ...
    %         iter, Maxiter, iter/Maxiter*100, node_count, best_length);
    %     
    %     % 更新进度显示
    %     subplot(2,3,3);
    %     set(progress_text, 'String', sprintf('IRRT*优化中...\n迭代: %d/%d\n节点数: %d\n当前最优: %.2f', ...
    %         iter, Maxiter, node_count, best_length));
    %     
    %     % 更新统计信息
    %     subplot(2,3,6);
    %     stats_str = sprintf('迭代次数: %d/%d\n树节点数: %d\n当前最优: %.2f\n初始长度: %.2f\n重选父节点: %d\n重布线: %d', ...
    %         iter, Maxiter, node_count, best_length, bug_apf_length, parent_select_count, rewire_count);
    %     set(stats_text, 'String', stats_str);
    %     
    %     % 更新树可视化
    %     update_tree_visualization(Tv_x, Tv_y, Tv_z, Tv_pre, node_count, rrt_tree_plot, rrt_nodes_plot);
    %     
    %     drawnow limitrate;
    % end
    if rand<0.1 &&b1<lb
        if b1==2
            pre_node = 1;
        end
    new_x = b_x(b1);
    new_y = b_y(b1);
    new_z = b_z(b1);   
    nearest_idx = pre_node;
    pre_node = node_count+1;
    else
    %% 优化的采样策略 - 椭球采样
    cb = max(best_length/2, 1);
    vec = goalPoint - startPoint;
    vec_length = norm(vec);
    if vec_length > 0
        vec_unit = vec / vec_length;
    else
        vec_unit = [0, 0, 1];
    end
    
    % 构建正交基
    if abs(vec_unit(3)) > 0.9
        temp = [0, 1, 0];
    else
        temp = [0, 0, 1];
    end
    ux = cross(vec_unit, temp); 
    if norm(ux) > 0
        ux = ux / norm(ux);
    else
        ux = [1, 0, 0];
    end
    uy = cross(vec_unit, ux); 
    if norm(uy) > 0
        uy = uy / norm(uy);
    else
        uy = [0, 1, 0];
    end
    
    u = rand^(1/3);
    theta = 2 * pi * rand;
    phi = pi * rand;
    
    cs = sqrt(max(cb^2 - norm(goalPoint-startPoint)^2/4, 1));
    center = (startPoint + goalPoint) / 2;
    
    local_x = u * sin(phi) * cos(theta) * cs;
    local_y = u * sin(phi) * sin(theta) * cs;
    local_z = u * cos(phi) * cb;
    
    randp_x = center(1) + local_x * ux(1) + local_y * uy(1) + local_z * vec_unit(1);
    randp_y = center(2) + local_x * ux(2) + local_y * uy(2) + local_z * vec_unit(2);
    randp_z = center(3) + local_x * ux(3) + local_y * uy(3) + local_z * vec_unit(3);
    
    % 边界检查
    randp_x = max(axisStart(1), min(axisLWH(1), randp_x));
    randp_y = max(axisStart(2), min(axisLWH(2), randp_y));
    randp_z = max(axisStart(3), min(axisLWH(3), randp_z));
    
    %% 障碍物检测
    valid_point = true;
    
    % 检测立方体障碍物
    for k = 1:length(cubeInfo.axisX)
        if randp_x >= cubeInfo.axisX(k) && randp_x < cubeInfo.axisX(k)+cubeInfo.lengthx(k)
            if randp_y >= cubeInfo.axisY(k) && randp_y < cubeInfo.axisY(k)+cubeInfo.lengthy(k)
                if randp_z >= cubeInfo.axisZ(k) && randp_z < cubeInfo.axisZ(k)+cubeInfo.lengthz(k)
                    valid_point = false;
                    break
                end
            end
        end
    end
    
    % 检测圆柱体障碍物
    if valid_point
        for k = 1:length(cylinderInfo.X)
            if sqrt((randp_x-cylinderInfo.X(k))^2+(randp_y-cylinderInfo.Y(k))^2) < cylinderInfo.radius(k)
                if randp_z >= cylinderInfo.Z(k) && randp_z < cylinderInfo.Z(k)+cylinderInfo.lengthZ(k)
                    valid_point = false;
                    break
                end
            end
        end
    end
    
    % 检测球体障碍物
    if valid_point
        for k = 1:length(sphereInfo.X)
            if sqrt((randp_x-sphereInfo.X(k))^2+(randp_y-sphereInfo.Y(k))^2+(randp_z-sphereInfo.Z(k))^2) < sphereInfo.radius(k)           
                valid_point = false;
                break
            end
        end
    end
    
    if ~valid_point
        continue;
    end
    
    %% 优化的最近邻搜索 - 线性搜索
    min_dist_sq = inf;
    nearest_idx = 1;
    
    % 优先搜索最近添加的节点（更可能接近目标）
    search_start =1;
    
    for idx = search_start:node_count
        dx = randp_x - Tv_x(idx);
        dy = randp_y - Tv_y(idx);
        dz = randp_z - Tv_z(idx);
        dist_sq = dx*dx + dy*dy + dz*dz;
        
        if dist_sq < min_dist_sq
            min_dist_sq = dist_sq;
            nearest_idx = idx;
        end
 end
    
    % 如果没找到足够近的，搜索整个树
    if min_dist_sq > (step*3)^2
        for idx = 1:min(search_start-1, node_count)
            dx = randp_x - Tv_x(idx);
            dy = randp_y - Tv_y(idx);
            dz = randp_z - Tv_z(idx);
            dist_sq = dx*dx + dy*dy + dz*dz;
            
            if dist_sq < min_dist_sq
                min_dist_sq = dist_sq;
                nearest_idx = idx;
            end
        end
    end
    
    r = sqrt(min_dist_sq);
    
    %% 生成新节点
    if r > 0
        fai = atan2(randp_y - Tv_y(nearest_idx), randp_x - Tv_x(nearest_idx));
        cos_theta = (randp_z - Tv_z(nearest_idx)) / r;
        cos_theta = max(min(cos_theta, 1), -1);
        theta = acos(cos_theta);
    else
        fai = 0;
        theta = pi/2;
    end
    
    new_x = Tv_x(nearest_idx) + step * sin(theta) * cos(fai);
    new_y = Tv_y(nearest_idx) + step * sin(theta) * sin(fai);
    new_z = Tv_z(nearest_idx) + step * cos(theta);
    end
    % 边界检查
    if new_x < axisStart(1) || new_x > axisLWH(1) || ...
       new_y < axisStart(2) || new_y > axisLWH(2) || ...
       new_z < axisStart(3) || new_z > axisLWH(3)
        continue;
    end
    
    %% 碰撞检测
    nearp.x = Tv_x(nearest_idx);
    nearp.y = Tv_y(nearest_idx);
    nearp.z = Tv_z(nearest_idx);
    newp.x = new_x;
    newp.y = new_y;
    newp.z = new_z;
    
    cubeFlag = cubecheck(nearp,cubeInfo,theta,fai,newp);
    cylinderFlag = cylindercheck(nearp,cylinderInfo,theta,fai,newp);
    sphereFlag = spherecheck(nearp,sphereInfo,theta,fai,newp);
    
    if cubeFlag || cylinderFlag || sphereFlag
        continue;
    end
    
    %% IRRT*节点扩展过程
    new_x_candidate = new_x;
    new_y_candidate = new_y; 
    new_z_candidate = new_z;
    
    % 步骤1: 在附近节点中寻找更优的父节点
    best_parent = nearest_idx;
    best_cost = Tv_cos(nearest_idx) + step;
    min_distance = step;

    % 搜索新节点附近的潜在父节点 - 使用线性搜索
    candidate_parents = [];
    search_window = min(500, node_count);
    
    for idx = 1:node_count
        if idx == nearest_idx
            continue;
        end
        
        dx_candidate = new_x_candidate - Tv_x(idx);
        dy_candidate = new_y_candidate - Tv_y(idx);
        dz_candidate = new_z_candidate - Tv_z(idx);
        dist_candidate = sqrt(dx_candidate*dx_candidate + dy_candidate*dy_candidate + dz_candidate*dz_candidate);
        
        if dist_candidate <= rewire_Thr
            candidate_parents(end+1) = idx;
        end
    end

    % 检查每个候选父节点
    for k = 1:length(candidate_parents)
        candidate_idx = candidate_parents(k);
        
        % 计算从候选父节点到新节点的代价
        dx_candidate = new_x_candidate - Tv_x(candidate_idx);
        dy_candidate = new_y_candidate - Tv_y(candidate_idx);
        dz_candidate = new_z_candidate - Tv_z(candidate_idx);
        dist_candidate = sqrt(dx_candidate*dx_candidate + dy_candidate*dy_candidate + dz_candidate*dz_candidate);
        
        candidate_cost = Tv_cos(candidate_idx) + dist_candidate;
        
        % 如果找到更优的父节点
        if candidate_cost < best_cost
            % 检查路径是否无碰撞
            startp_check.x = Tv_x(candidate_idx);
            startp_check.y = Tv_y(candidate_idx);
            startp_check.z = Tv_z(candidate_idx);
            endp_check.x = new_x_candidate;
            endp_check.y = new_y_candidate;
            endp_check.z = new_z_candidate;
            
            r_check = dist_candidate;
            if r_check > 0
                fai_check = atan2(endp_check.y - startp_check.y, endp_check.x - startp_check.x);
                cos_theta_check = (endp_check.z - startp_check.z) / r_check;
                cos_theta_check = max(min(cos_theta_check, 1), -1);
                theta_check = acos(cos_theta_check);
            else
                fai_check = 0;
                theta_check = 0;
            end
            
            cubeFlag_check = cubecheck(startp_check, cubeInfo, theta_check, fai_check, endp_check);
            cylinderFlag_check = cylindercheck(startp_check, cylinderInfo, theta_check, fai_check, endp_check);
            sphereFlag_check = spherecheck(startp_check, sphereInfo, theta_check, fai_check, endp_check);
            
            if ~(cubeFlag_check || cylinderFlag_check || sphereFlag_check)
                best_parent = candidate_idx;
                best_cost = candidate_cost;
                min_distance = dist_candidate;
                parent_select_count = parent_select_count + 1;
            end
        end
    end

    % 步骤2: 使用最优父节点添加新节点到树
    node_count = node_count + 1;
    if node_count > maxNodes
        expansion = 1000;
        Tv_x = [Tv_x; zeros(expansion, 1)];
        Tv_y = [Tv_y; zeros(expansion, 1)];
        Tv_z = [Tv_z; zeros(expansion, 1)];
        Tv_pre = [Tv_pre; zeros(expansion, 1)];
        Tv_cos = [Tv_cos; zeros(expansion, 1)];
        maxNodes = maxNodes + expansion;
    end

    Tv_x(node_count) = new_x_candidate;
    Tv_y(node_count) = new_y_candidate;
    Tv_z(node_count) = new_z_candidate;
    Tv_pre(node_count) = best_parent;
    Tv_cos(node_count) = best_cost;
    
    % 步骤3: 重布线 - 优化新节点附近的现有节点

    
    for idx = 1:node_count
        if idx == node_count || idx == best_parent
            continue;
        end
        
        dx = Tv_x(idx) - new_x_candidate;
        dy = Tv_y(idx) - new_y_candidate;
        dz = Tv_z(idx) - new_z_candidate;
        dist = sqrt(dx*dx + dy*dy + dz*dz);
        
        if dist <= rewire_Thr
            new_cost = Tv_cos(node_count) + dist;
            
            if new_cost < Tv_cos(idx)
                % 碰撞检测：从新节点到候选节点
                startp_check.x = new_x_candidate;
                startp_check.y = new_y_candidate;
                startp_check.z = new_z_candidate;
                endp_check.x = Tv_x(idx);
                endp_check.y = Tv_y(idx);
                endp_check.z = Tv_z(idx);
                
                r_check = dist;
                if r_check > 0
                    fai_check = atan2(endp_check.y - startp_check.y, endp_check.x - startp_check.x);
                    cos_theta_check = (endp_check.z - startp_check.z) / r_check;
                    cos_theta_check = max(min(cos_theta_check, 1), -1);
                    theta_check = acos(cos_theta_check);
                else
                    fai_check = 0;
                    theta_check = 0;
                end
                
                cubeFlag_check = cubecheck(startp_check, cubeInfo, theta_check, fai_check, endp_check);
                cylinderFlag_check = cylindercheck(startp_check, cylinderInfo, theta_check, fai_check, endp_check);
                sphereFlag_check = spherecheck(startp_check, sphereInfo, theta_check, fai_check, endp_check);
                
                if ~(cubeFlag_check || cylinderFlag_check || sphereFlag_check)
                    % 重布线：将候选节点的父节点设置为新节点
                    old_parent = Tv_pre(idx);
                    old_cost_val = Tv_cos(idx);
                    
                    Tv_pre(idx) = node_count;
                    Tv_cos(idx) = new_cost;
                    
                    % 代价传播
                    cost_diff = new_cost - old_cost_val;
                    queue = idx;
                    
                    while ~isempty(queue)
                        current_idx = queue(1);
                        queue(1) = [];
                        
                        % 找到所有直接子节点
                        for child_idx = 1:node_count
                            if Tv_pre(child_idx) == current_idx
                                Tv_cos(child_idx) = Tv_cos(child_idx) + cost_diff;
                                queue(end+1) = child_idx;
                            end
                        end
                    end
                    
                    rewire_count = rewire_count + 1;
                end
            end
        end
    end
    
    %% 目标检查
    dist_to_goal = norm([new_x_candidate, new_y_candidate, new_z_candidate] - goalPoint);
    
    if dist_to_goal < Thr
        if ~findpath
            findpath = 1;
            % 手动将目标点加入到树中
            node_count = node_count + 1;
            if node_count > maxNodes
                expansion = 1000;
                Tv_x = [Tv_x; zeros(expansion, 1)];
                Tv_y = [Tv_y; zeros(expansion, 1)];
                Tv_z = [Tv_z; zeros(expansion, 1)];
                Tv_pre = [Tv_pre; zeros(expansion, 1)];
                Tv_cos = [Tv_cos; zeros(expansion, 1)];
                maxNodes = maxNodes + expansion;
            end
            
            Goal_index = node_count;
            Tv_x(node_count) = goalPoint(1);          
            Tv_y(node_count) = goalPoint(2); 
            Tv_z(node_count) = goalPoint(3);
            Tv_pre(node_count) = node_count - 1;  
            Tv_cos(node_count) = Tv_cos(node_count - 1) + dist_to_goal; 
            best_length = min(best_length, Tv_cos(Goal_index));
%             fprintf('首次找到路径! 长度: %.2f\n', Tv_cos(Goal_index));
        else
            % 计算实际路径长度
            current_path_indices = [];
            temp_idx = node_count;
            while temp_idx > 0
                current_path_indices = [temp_idx, current_path_indices];
                temp_idx = Tv_pre(temp_idx);
                if length(current_path_indices) > node_count
                    break;
                end
            end
            
            current_cost = 0;
            for k = 2:length(current_path_indices)
                idx1 = current_path_indices(k-1);
                idx2 = current_path_indices(k);
                segment_length = norm([Tv_x(idx2)-Tv_x(idx1), Tv_y(idx2)-Tv_y(idx1), Tv_z(idx2)-Tv_z(idx1)]);
                current_cost = current_cost + segment_length;
            end
            
            % 加上最后一段到目标的距离
            current_cost = current_cost + dist_to_goal;
            
            if current_cost < best_length
                best_length = current_cost;
%                 fprintf('找到更好路径! 实际长度: %.2f\n', best_length);
            end
        end
    end
end

%% 提取并绘制最终路径 - 注释掉可视化部分
if Goal_index > 0 && Goal_index <= node_count
    path_indices = [];
    current_idx = Goal_index;
    
    while current_idx > 0
        path_indices = [current_idx, path_indices];
        current_idx = Tv_pre(current_idx);
        if length(path_indices) > node_count
            break;
        end
    end
    
    % 计算最终路径长度
    final_path_length = 0;
    for k = 2:length(path_indices)
        idx1 = path_indices(k-1);
        idx2 = path_indices(k);
        segment_length = norm([Tv_x(idx2)-Tv_x(idx1), Tv_y(idx2)-Tv_y(idx1), Tv_z(idx2)-Tv_z(idx1)]);
        final_path_length = final_path_length + segment_length;
    end
    
    % 绘制最终路径 - 注释掉
    % subplot(2,3,[1,2,4,5]);
    % set(final_path_plot, 'XData', Tv_x(path_indices), 'YData', Tv_y(path_indices), 'ZData', Tv_z(path_indices));
    % 
    % % 在终点添加特殊标记 - 注释掉
    % scatter3(goalPoint(1), goalPoint(2), goalPoint(3), 150, 'y', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', '优化目标');
    % 
    % % 更新图例 - 注释掉
    % legend('起点', '终点', '立方体障碍物', '圆柱障碍物', '球体障碍物', ...
    %        'BUG-APF路径', '当前位置', '合力方向', 'RRT*树', 'RRT*节点', '最终路径', '优化目标', ...
    %        'Location', 'northeast');
    
%     fprintf('最终路径长度: %.2f (优化前: %.2f)\n', final_path_length, bug_apf_length);
%     improvement = ((bug_apf_length - final_path_length) / bug_apf_length) * 100;
%     fprintf('优化效果: %.2f%% 改进\n', improvement);
else
    % 如果没有找到更好路径，使用BUG-APF路径
    path_indices = 1:length(b_x);
    final_path_length = bug_apf_length;
    improvement = 0;
%     fprintf('未找到更好路径，使用原始BUG-APF路径\n');
end

%% 辅助函数：更新树可视化 - 注释掉函数定义
% function update_tree_visualization(Tv_x, Tv_y, Tv_z, Tv_pre, node_count, tree_plot, nodes_plot)
%     % 只显示最近添加的节点以提高性能
%     start_idx = max(1, node_count - 800);
%     tree_x = [];
%     tree_y = [];
%     tree_z = [];
%     
%     for i = start_idx:node_count
%         if Tv_pre(i) > 0 && Tv_pre(i) <= node_count
%             tree_x = [tree_x, Tv_x(Tv_pre(i)), Tv_x(i), NaN];
%             tree_y = [tree_y, Tv_y(Tv_pre(i)), Tv_y(i), NaN];
%             tree_z = [tree_z, Tv_z(Tv_pre(i)), Tv_z(i), NaN];
%         end
%     end
%     
%     set(tree_plot, 'XData', tree_x, 'YData', tree_y, 'ZData', tree_z);
%     set(nodes_plot, 'XData', Tv_x(1:node_count), 'YData', Tv_y(1:node_count), 'ZData', Tv_z(1:node_count));
% end

%% 最终状态显示和统计
a = final_path_length;
t = toc;
end
% 更新最终状态显示 - 注释掉
% subplot(2,3,3);
% set(progress_text, 'String', sprintf('算法完成!\n总时间: %.2f秒\n最终路径长度: %.2f\n优化效果: %.2f%%\n重选父节点: %d次\n重布线: %d次', ...
%     t, final_path_length, improvement, parent_select_count, rewire_count));
% 
% subplot(2,3,6);
% final_stats = sprintf('总运行时间: %.2f秒\n最终路径长度: %.2f\n初始路径长度: %.2f\n优化效果: %.2f%%\n总节点数: %d\nBUG-APF步数: %d\nRRT*迭代: %d\n重选父节点: %d次\n重布线: %d次', ...
%     t, final_path_length, bug_apf_length, improvement, node_count, length(b_x), Maxiter, parent_select_count, rewire_count);
% set(stats_text, 'String', final_stats);
% 
% % 添加完成提示 - 注释掉
% subplot(2,3,[1,2,4,5]);
% text(axisLWH(1)/2, axisLWH(2)/2, axisLWH(3)+10, ...
%     sprintf('算法完成! 时间: %.2f秒\n路径优化: %.2f%%', t, improvement), ...
%     'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold', ...
%     'BackgroundColor', 'white', 'EdgeColor', 'black');
% 
% % 更新树可视化 - 注释掉
% update_tree_visualization(Tv_x, Tv_y, Tv_z, Tv_pre, node_count, rrt_tree_plot, rrt_nodes_plot);
% 
% drawnow;

% fprintf('算法完成！总运行时间: %.2f 秒\n', t);
% fprintf('最终路径长度: %.2f (优化前: %.2f)\n', final_path_length, bug_apf_length);
% fprintf('优化效果: %.2f%%\n', improvement);
% fprintf('总节点数: %d\n', node_count);
% fprintf('重选父节点成功次数: %d\n', parent_select_count);
% fprintf('重布线成功次数: %d\n', rewire_count);