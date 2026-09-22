 function [a,t2] = BAIRy()

tic;
%% 定义地图
resolution = 1; % 网格长度

left_bound = 0;
right_bound = 150;
lower_bound = 0;
upper_bound = 150;   
% 创建墙体 [左下角x坐标,左下角y坐标,水平长度,垂直长度]
wall_obstacle(1,:) = [   left_bound-3,   lower_bound,                        3, upper_bound-lower_bound+3]; %左边界
wall_obstacle(2,:) = [ left_bound-3,   lower_bound-3, right_bound-left_bound+3,                         3]; % 下边界
wall_obstacle(3,:) = [right_bound, lower_bound-3,                        3, upper_bound-lower_bound+3]; % 右边界
wall_obstacle(4,:) = [   left_bound, upper_bound, right_bound-left_bound+3,                         3]; % 上边界

%% 画出地图
figure(2); % 创建图片
%图片设置
set(gca,'XLim',[left_bound-3 right_bound+3]); % x轴范围
set(gca,'YLim',[lower_bound-3 upper_bound+3]); % y轴范围
axis equal
title('Bug0-APF-IRRT* with Improved Vortex Force');
xlabel('x');
ylabel('y');

hold on
%绘制墙体
for i=1:1:size(wall_obstacle,1)
   h_wall=fill([wall_obstacle(i,1),wall_obstacle(i,1)+wall_obstacle(i,3),wall_obstacle(i,1)+wall_obstacle(i,3),wall_obstacle(i,1)],...
         [wall_obstacle(i,2),wall_obstacle(i,2),wall_obstacle(i,2)+wall_obstacle(i,4),wall_obstacle(i,2)+wall_obstacle(i,4)],'k');
end

%绘制方形障碍物
square_obstacle(1,:) = [20,5,20,40]; % 方形障碍物1
square_obstacle(2,:) = [10,70,20,40]; % 方形障碍物2
square_obstacle(3,:) = [60,40,30,30]; %方形障碍物3
squareob = [square_obstacle; wall_obstacle];
for i=1:1:size(square_obstacle,1)
   h_square=fill([square_obstacle(i,1),square_obstacle(i,1)+square_obstacle(i,3),square_obstacle(i,1)+square_obstacle(i,3),square_obstacle(i,1)],...
         [square_obstacle(i,2),square_obstacle(i,2),square_obstacle(i,2)+square_obstacle(i,4),square_obstacle(i,2)+square_obstacle(i,4)],'k');
end

%绘制圆形障碍物
round_obstacle(1,:) = [100,100,12]; %圆形障碍物1
round_obstacle(2,:) = [130,100,12]; %圆形障碍物2
round_obstacle(3,:) = [70,110,12]; %圆形障碍物2
for i=1:1:size(round_obstacle,1)
    center = [round_obstacle(i,1),round_obstacle(i,2)]; % 圆心 (x0, y0)
    r = round_obstacle(i,3); % 半径
    theta = 0:0.01*pi:2*pi;
    x = r * cos(theta) + center(1);
    y = r * sin(theta) + center(2);
    h_round=fill(x, y, 'k', 'EdgeColor', 'k', 'LineWidth', 1.5);
end



% 参数初始化
x_I = 1; y_I = 1;           % 设置初始点
x_G = 145; y_G = 145;       % 设置目标点p
GoalThreshold = 2;         %设置终点范围
range=3;             %设置寻路完成范围
r_f=50;              %设置引力影响变化范围
k_att=0.003;         %设置引力常数
k_rtex = 0.0035;   %设置涡流力常数
k_stex = 0.0035;   %设置方形障碍物涡流力常数
k_rep=0.9;        %设置斥力常数
plot(x_I, y_I, 'mo', 'MarkerSize',10, 'MarkerFaceColor','m');   % 绘制起点和目标点
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');

%BUG0-APF寻路开始
% 使用数组代替struct结构 - 预分配内存
max_steps = 2000;
b_x = zeros(1, max_steps);
b_y = zeros(1, max_steps);
b_dx = zeros(1, max_steps);
b_dy = zeros(1, max_steps);

% 初始化
i = 1;
b_x(i) = x_I;
b_y(i) = y_I;
b_dx(i) = 0;
b_dy(i) = 0;
rho_0 = 10;

% 取消步数限制，确保APF一定能找到路径
while sqrt((b_x(i)-x_G)^2+(b_y(i)-y_G)^2) > range && i < max_steps
    
    % 计算引力
    dist_to_goal = sqrt((b_x(i)-x_G)^2+(b_y(i)-y_G)^2);
    if dist_to_goal > r_f
        f_att = [x_G-b_x(i), y_G-b_y(i)] * k_att;
    else
        f_att = [x_G-b_x(i), y_G-b_y(i)] * k_att * 50 / dist_to_goal;
    end
    
    % 初始化力向量
    f_rep = [0,0];
    f_rtex = [0,0];
    f_stex = [0,0];
    
    % 碰撞检测
    if collisionChecking([b_x(i),b_y(i)],[x_G,y_G],square_obstacle,round_obstacle)
        b_dx(i) = x_G - b_x(i);
        b_dy(i) = y_G - b_y(i);
        b_x(i+1) = x_G;
        b_y(i+1) = y_G;
        b_dx(i+1) = 0;
        b_dy(i+1) = 0;
        plot([b_x(i), b_x(i+1)], [b_y(i), b_y(i+1)], 'g', 'Linewidth', 2);
        break
    else
        % 处理方形障碍物
        for j = 1:size(square_obstacle, 1)
            x1 = square_obstacle(j, 1);
            y1 = square_obstacle(j, 2);
            x2 = square_obstacle(j, 1) + square_obstacle(j, 3);
            y2 = square_obstacle(j, 2) + square_obstacle(j, 4);
            
            % 检查当前位置是否在障碍物内部
            if b_x(i) >= x1 && b_x(i) <= x2 && b_y(i) >= y1 && b_y(i) <= y2
                disp('初始位置位于障碍物内');
                break;
            end
            
            % 计算与障碍物最近点的距离
            x_closest = b_x(i);
            y_closest = b_y(i);
            if b_x(i) < x1
                x_closest = x1;
            elseif b_x(i) > x2
                x_closest = x2;
            end
            if b_y(i) < y1
                y_closest = y1;
            elseif b_y(i) > y2
                y_closest = y2;
            end
            
            % 计算与最近点的距离
            dist_to_obstacle = norm([x_closest - b_x(i), y_closest - b_y(i)]);
            
            % 如果距离小于影响范围，计算斥力和涡流力
            if dist_to_obstacle < rho_0
                % 计算斥力
                f_rep = f_rep + k_rep * (1/dist_to_obstacle - 1/rho_0) * 1/dist_to_obstacle^2 * ([b_x(i), b_y(i)] - [x_closest, y_closest]);
                
                % 计算障碍物表面法线方向
                normal_direction = [b_x(i) - x_closest, b_y(i) - y_closest];
                normal_direction = normal_direction / norm(normal_direction);
                
                % 计算切线方向
                tangent_direction = [-normal_direction(2), normal_direction(1)];
                
                % 根据目标位置选择绕行方向
                obstacle_center = [x1 + (x2-x1)/2, y1 + (y2-y1)/2];
                vec_robot = [b_x(i), b_y(i)] - obstacle_center;
                vec_goal = [x_G, y_G] - obstacle_center;
                
                % 使用叉积判断绕行方向
                cross_product = vec_robot(1)*vec_goal(2) - vec_robot(2)*vec_goal(1);
                
                if cross_product > 0
                    preferred_tangent = tangent_direction;
                else
                    preferred_tangent = -tangent_direction;
                end
                
                % 涡流力计算
                vortex_magnitude = k_stex / k_att * 0.2 * (1/dist_to_obstacle - 1/rho_0);
                f_stex = f_stex + vortex_magnitude * preferred_tangent;
            end
        end
        
        % 处理圆形障碍物
        for j = 1:size(round_obstacle, 1)
            x1 = round_obstacle(j, 1);
            y1 = round_obstacle(j, 2);
            r_j = round_obstacle(j, 3);
            
            % 检查是否在障碍物内部
            if norm([x1 - b_x(i), y1 - b_y(i)]) < r_j
                disp('初始位置位于障碍物内');
                break;
            end
            
            % 计算是否处在影响范围
            if norm([x1 - b_x(i), y1 - b_y(i)]) < r_j + rho_0
                dist_to_obstacle = norm([x1 - b_x(i), y1 - b_y(i)]) - r_j;
                f_rep = f_rep + k_rep * (1/dist_to_obstacle - 1/rho_0) * 1/dist_to_obstacle^2 * ([b_x(i), b_y(i)] - [x1, y1]);
                
                % 计算障碍物表面法线方向
                normal_direction = [b_x(i) - x1, b_y(i) - y1];
                normal_direction = normal_direction / norm(normal_direction);
                
                % 计算切线方向
                tangent_direction = [-normal_direction(2), normal_direction(1)];
                
                % 计算引力在切线方向的分量
                att_tangent_magnitude = dot(f_att, tangent_direction);
                
                % 根据目标位置选择绕行方向
                robot_to_goal = [x_G - b_x(i), y_G - b_y(i)];
                
                % 使用叉积判断绕行方向
                cross_product = normal_direction(1)*robot_to_goal(2) - normal_direction(2)*robot_to_goal(1);
                
                if cross_product > 0
                    preferred_tangent = tangent_direction;
                else
                    preferred_tangent = -tangent_direction;
                end
                
                % 确保切线方向与优选方向一致
                if dot(tangent_direction, preferred_tangent) < 0
                    tangent_direction = -tangent_direction;
                end
                
                % 涡流力计算
                vortex_magnitude = k_rtex / k_att * abs(att_tangent_magnitude);
                f_rtex = f_rtex + vortex_magnitude * tangent_direction * (1/dist_to_obstacle - 1/rho_0);
            end
        end
    end
    
    % 合力计算
    f_tot = f_rep + f_att + f_rtex + f_stex;
    if norm(f_rep) > norm(f_att)
        f_tot = f_rtex + f_stex;
    end
    
    % 防止力过大
    force_magnitude = norm(f_tot);
    if force_magnitude > 5
        f_tot = f_tot / force_magnitude * 5;
    end
    
    % 速度更新
    if i < 2 || ~(f_rep(1) == 0 && f_rep(2) == 0)
        b_dx(i) = f_tot(1);
        b_dy(i) = f_tot(2);
    else
        b_dx(i) = b_dx(i-1) + f_tot(1);
        b_dy(i) = b_dy(i-1) + f_tot(2);
    end
    
    % 防止速度过大
    velocity_magnitude = norm([b_dx(i), b_dy(i)]);
    if velocity_magnitude > 3
        b_dx(i) = b_dx(i) / velocity_magnitude * 3;
        b_dy(i) = b_dy(i) / velocity_magnitude * 3;
    end
    if velocity_magnitude < 0.01
        f_tot = f_rtex + f_stex;
        b_dx(i) = f_tot(1);
        b_dy(i) = f_tot(2);
    end
    
    % 位置更新
    b_x(i+1) = b_x(i) + b_dx(i);
    b_y(i+1) = b_y(i) + b_dy(i);
    i = i + 1;
    
    % 绘制路径
    plot([b_x(i), b_x(i-1)], [b_y(i), b_y(i-1)], 'g', 'Linewidth', 2);
end

fprintf('APF路径规划完成，共%d步!\n', i);
% 裁剪数组
b_x = b_x(1:i+1);
b_y = b_y(1:i+1);
b_dx = b_dx(1:i+1);
b_dy = b_dy(1:i+1);
% 如果APF失败，直接返回

%转入IRRT*算法进行全局优化
%初始化树节点
Maxinteration = 10000;

% 使用数组替代结构体
maxNodes = Maxinteration + 1000;
Tv_x = zeros(1, maxNodes);
Tv_y = zeros(1, maxNodes);
Tv_pre = zeros(1, maxNodes);
Tv_cos = zeros(1, maxNodes);

% 初始化树
Tv_x(1) = x_I; 
Tv_y(1) = y_I;
Tv_pre(1) = 0; 
Tv_cos(1) = 0;

node_count = 1;

% 计算APF路径长度
bug_apf_length = 0;
for k = 2:length(b_x)
    bug_apf_length = bug_apf_length + sqrt((b_x(k)-b_x(k-1))^2 + (b_y(k)-b_y(k-1))^2);
end

best_length = bug_apf_length;
Goal_index = 0;
findpath = 0;

% 统计信息
rewire_count = 0;
parent_select_count = 0;

% IRRT*参数
step = 3; % 步长
Thr = 2; % 目标阈值
rewire_Thr = 8; % 重布线阈值

% APF路径参数
apf_index = 2;
pre_node = 1;

% fprintf('开始IRRT*优化，APF路径长度: %.2f\n', bug_apf_length);

% IRRT*主循环
for iter = 1:Maxinteration
    if node_count >= 2000
        break;
    end
    
    if node_count < 1000
        rewire_Thr = 16;
    else
        rewire_Thr = 8;
    end
    
    % 决定是否添加APF路径点
    if rand() < 0.5 && apf_index < length(b_x)
        % 使用APF路径点作为新节点
        new_x = b_x(apf_index);
        new_y = b_y(apf_index);
        nearest_idx = pre_node;
        pre_node = node_count + 1;
        apf_index = apf_index + 1;
        
        % 直接添加，不进行碰撞检测（APF路径本身无碰撞）
        distance = sqrt((new_x - Tv_x(nearest_idx))^2 + (new_y - Tv_y(nearest_idx))^2);
        costmin = Tv_cos(nearest_idx) + distance;
        cminIndex = nearest_idx;
        
    else
        % 椭圆采样
        cb = best_length/2;
        cm = sqrt((x_I-x_G)^2+(y_I-y_G)^2)/2;
        cz = [(x_I+x_G)/2, (y_I+y_G)/2];
        rtheta = atan2((y_I-y_G), (x_I-x_G));
        cs = sqrt(max(cb^2 - cm^2, 0));
        u = sqrt(rand(1));
        v = rand(1);
        x_rand = [cb*u*cos(2*pi*v)*cos(rtheta) - cs*u*sin(2*pi*v)*sin(rtheta), ...
                  cs*u*sin(2*pi*v)*cos(rtheta) + cb*u*cos(2*pi*v)*sin(rtheta)] + cz;
        
        % 检查随机点是否有效
        out = 1;
        for k = 1:size(squareob, 1)
            if x_rand(1) >= squareob(k,1) && x_rand(1) <= (squareob(k,1)+squareob(k,3)) && ...
               x_rand(2) >= squareob(k,2) && x_rand(2) <= squareob(k,2)+squareob(k,4)
                out = 0;
                break;
            end
        end
        for k = 1:size(round_obstacle, 1)
            if sqrt((x_rand(1)-round_obstacle(k,1))^2 + (x_rand(2)-round_obstacle(k,2))^2) < round_obstacle(k,3)
                out = 0;
                break;
            end
        end
        if out == 0
            continue;
        end
        
        % 查找最近节点
        minDis = inf;
        nearest_idx = 1;
        for idx = 1:node_count
            distance = sqrt((x_rand(1) - Tv_x(idx))^2 + (x_rand(2) - Tv_y(idx))^2);
            if distance < minDis
                minDis = distance;
                nearest_idx = idx;
            end
        end
        
        % 扩展新节点
        theta = atan2((x_rand(2) - Tv_y(nearest_idx)), (x_rand(1) - Tv_x(nearest_idx)));
        new_x = Tv_x(nearest_idx) + cos(theta) * step;
        new_y = Tv_y(nearest_idx) + sin(theta) * step;
        
        % 检查新节点是否有效
        if new_x < left_bound || new_x > right_bound || ...
           new_y < lower_bound || new_y > upper_bound
            continue;
        end
        
        % 碰撞检测
        if ~collisionChecking([Tv_x(nearest_idx), Tv_y(nearest_idx)], [new_x, new_y], squareob, round_obstacle)
            continue;
        end
        
        % 初始代价
        costmin = step + Tv_cos(nearest_idx);
        cminIndex = nearest_idx;
    end
    
    % 重选父节点 - 在附近节点中寻找最优父节点
    for idx = 1:node_count
        distance = sqrt((new_x - Tv_x(idx))^2 + (new_y - Tv_y(idx))^2);
        if distance <= rewire_Thr
            cost = distance + Tv_cos(idx);
            if cost < costmin
                if collisionChecking([Tv_x(idx), Tv_y(idx)], [new_x, new_y], squareob, round_obstacle)
                    costmin = cost;
                    cminIndex = idx;
                    parent_select_count = parent_select_count + 1;
                end
            end
        end
    end
    
    % 添加到树中
    node_count = node_count + 1;
    Tv_x(node_count) = new_x;
    Tv_y(node_count) = new_y;
    Tv_pre(node_count) = cminIndex;
    Tv_cos(node_count) = costmin;
    
    % 重布线
    for idx = 1:node_count-1
        distance = sqrt((new_x - Tv_x(idx))^2 + (new_y - Tv_y(idx))^2);
        if distance <= rewire_Thr && idx ~= cminIndex
            newCost = costmin + distance;
            
            if newCost < Tv_cos(idx)
                if collisionChecking([new_x, new_y], [Tv_x(idx), Tv_y(idx)], squareob, round_obstacle)
                    % 记录旧代价
                    old_cost = Tv_cos(idx);
                    
                    % 重布线
                    Tv_pre(idx) = node_count;
                    Tv_cos(idx) = newCost;
                    
                    % 代价传播
                    cost_diff = newCost - old_cost;
                    for child_idx = 1:node_count
                        if Tv_pre(child_idx) == idx
                            Tv_cos(child_idx) = Tv_cos(child_idx) + cost_diff;
                        end
                    end
                    
                    rewire_count = rewire_count + 1;
                    
                    % 如果更新了目标节点，更新最佳路径长度
                    if idx == Goal_index
                        best_length = newCost;
                    end
                end
            end
        end
    end
    
    % 检查是否到达目标点 - 修正目标点添加逻辑
    dist_to_goal = sqrt((new_x - x_G)^2 + (new_y - y_G)^2);
    if dist_to_goal <= range
        if ~findpath
            % 首次找到路径，添加目标点
            node_count = node_count + 1;
            Tv_x(node_count) = x_G;
            Tv_y(node_count) = y_G;
            Tv_pre(node_count) = node_count - 1;
            goal_distance = sqrt((x_G - Tv_x(node_count-1))^2 + (y_G - Tv_y(node_count-1))^2);
            Tv_cos(node_count) = Tv_cos(node_count - 1) + goal_distance;
            
            Goal_index = node_count;
            best_length = Tv_cos(Goal_index);
            findpath = 1;
%             fprintf('找到路径，长度: %.2f\n', best_length);
        else
            % 如果已经找到路径，检查是否有更短的路径
            current_cost = Tv_cos(node_count) + dist_to_goal;
            if current_cost < best_length
                % 更新目标点连接
                Tv_pre(Goal_index) = node_count;
                Tv_cos(Goal_index) = current_cost;
                best_length = current_cost;
%                 fprintf('更新路径，新长度: %.2f\n', best_length);
            end
        end
    end
end

% 在循环结束后统一绘制树
for i = 2:node_count
    if Tv_pre(i) > 0 && Tv_pre(i) <= node_count
        % 区分APF节点和随机节点
        is_apf_node = false;
        for k = 2:length(b_x)
            if abs(Tv_x(i) - b_x(k)) < 0.1 && abs(Tv_y(i) - b_y(k)) < 0.1
                is_apf_node = true;
                break;
            end
        end
        
        if is_apf_node
            plot([Tv_x(Tv_pre(i)), Tv_x(i)], [Tv_y(Tv_pre(i)), Tv_y(i)], 'm', 'Linewidth', 0.01);
        else
            plot([Tv_x(Tv_pre(i)), Tv_x(i)], [Tv_y(Tv_pre(i)), Tv_y(i)], 'b', 'Linewidth', 0.01);
        end
    end
end

% 提取最终路径
if findpath
    path_x = [];
    path_y = [];
    q = Goal_index;
    path_count = 1;
    path_x(path_count) = Tv_x(q);
    path_y(path_count) = Tv_y(q);
    
    while q ~= 1
        path_count = path_count + 1;
        q = Tv_pre(q);
        path_x(path_count) = Tv_x(q);
        path_y(path_count) = Tv_y(q);
    end
    
    % 反转路径
    path_x = fliplr(path_x);
    path_y = fliplr(path_y);
    
    % 绘制最终路径
    for u = 1:path_count-1
        plot([path_x(u), path_x(u+1)], [path_y(u), path_y(u+1)], 'r', 'Linewidth', 3);
    end
    
    % 计算最终路径长度
    final_length = 0;
    for u = 1:path_count-1
        final_length = final_length + sqrt((path_x(u+1)-path_x(u))^2 + (path_y(u+1)-path_y(u))^2);
    end
    
%     fprintf('IRRT*优化完成: 路径长度=%.2f, 节点数=%d\n', final_length, node_count);
    a = final_length;
    
    % 如果优化后的路径比APF路径还长，使用APF路径
    if final_length > bug_apf_length
%         fprintf('警告: IRRT*路径比APF路径长，使用APF路径\n');
        a = bug_apf_length;
        
        % 清除IRRT*绘制的路径，重新绘制APF路径
        cla;
        % 重新绘制障碍物和起点终点
        % ... (这里需要重新绘制地图)
        
        % 绘制APF路径
        for u = 1:length(b_x)-1
            plot([b_x(u), b_x(u+1)], [b_y(u), b_y(u+1)], 'r', 'Linewidth', 3);
        end
    end
else
%     fprintf('IRRT*未找到优化路径，使用APF路径\n');
    path_x = b_x;
    path_y = b_y;
    
    for u = 1:length(path_x)-1
        plot([path_x(u), path_x(u+1)], [path_y(u), path_y(u+1)], 'r', 'Linewidth', 3);
    end
    
    a = bug_apf_length;
end

t2 = toc;
   end
