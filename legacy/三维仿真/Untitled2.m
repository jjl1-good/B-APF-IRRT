function [a, t] = IRT3D_visualized()
clc
clear all;

tic
%% 性能优化设置
feature('accel','on');
warning('off', 'MATLAB:colon:nonIntegerIndex');

%% 定义变量
axisStart = [0 0 0];
axisLWH=[200 200 150];

%% 定义障碍物
cubeInfo.exist=0;
cylinderInfo.exist=0;
sphereInfo.exist=0;

%% 定义初始点
pathPoint= [26 26 1; 175 175 150];
update_count = 0;      
cubeInfo = createCube(cubeInfo);
cylinderInfo = createcylinder(cylinderInfo);
sphereInfo =createSphere(sphereInfo);
influencerange = 10;

%% 创建可视化窗口
figure(1);
set(gcf, 'Position', [100, 100, 1400, 800]);

% 主路径规划视图
subplot(2,3,[1,2,4,5]);
hold on;
grid on;
axis equal;
axis([0 200 0 200 0 150]);
view(45, 30);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('三维路径规划 - IRRT*算法', 'FontSize', 12, 'FontWeight', 'bold');

% 绘制障碍物
colorMatCube = [1 0 0]; 
colorMatCylinder = [0 1 0];
colorMatSphere = [0 0 1];
pellucidity = 0.3;

drawCube(cubeInfo,colorMatCube,pellucidity);
drawcylinder(cylinderInfo,colorMatCylinder,pellucidity);
drawsphere(sphereInfo,colorMatSphere,pellucidity);

% 绘制起点和终点
startPoint = pathPoint(1,:);
goalPoint = pathPoint(2,:);
scatter3(startPoint(1), startPoint(2), startPoint(3), 120, 'g', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
text(startPoint(1), startPoint(2), startPoint(3), '  起点', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'g');

scatter3(goalPoint(1), goalPoint(2), goalPoint(3), 120, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
text(goalPoint(1), goalPoint(2), goalPoint(3), '  终点', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');

% 创建图例
legend_items = [];
legend_items(1) = scatter3(NaN, NaN, NaN, 100, 'g', 'filled', 'DisplayName', '起点');
legend_items(2) = scatter3(NaN, NaN, NaN, 100, 'r', 'filled', 'DisplayName', '终点');
legend_items(3) = patch(NaN, NaN, 'r', 'FaceAlpha', 0.3, 'DisplayName', '立方体障碍物');
legend_items(4) = patch(NaN, NaN, 'g', 'FaceAlpha', 0.3, 'DisplayName', '圆柱障碍物');
legend_items(5) = patch(NaN, NaN, 'b', 'FaceAlpha', 0.3, 'DisplayName', '球体障碍物');

% 添加IRRT*可视化元素
rrt_tree_plot = plot3(NaN, NaN, NaN, 'b-', 'LineWidth', 0.5, 'DisplayName', 'RRT*树', 'Color', [0.7 0.7 1]);
rrt_nodes_plot = scatter3(NaN, NaN, NaN, 20, 'c', 'filled', 'MarkerFaceAlpha', 0.5, 'DisplayName', 'RRT*节点');
final_path_plot = plot3(NaN, NaN, NaN, 'r-', 'LineWidth', 4, 'DisplayName', '最终路径');

legend(legend_items, 'Location', 'northeast');
drawnow;

%% 进度显示子图
subplot(2,3,3);
progress_text = text(0.1, 0.5, '准备开始 IRRT* 路径规划...', 'FontSize', 12, 'FontWeight', 'bold');
axis off;
title('算法状态', 'FontSize', 12, 'FontWeight', 'bold');

%% 统计信息子图
subplot(2,3,6);
stats_text = text(0.1, 0.7, '', 'FontSize', 10);
axis off;
title('运行统计', 'FontSize', 12, 'FontWeight', 'bold');

%% 改进的IRRT*算法 - 使用完整的碰撞检测
fprintf('开始IRRT*优化...\n');

% IRRT*参数
Maxiter = 50000; % 最大迭代次数
step = 6; % 步长
Thr = 8; % 目标阈值
rewire_Thr = 15; % 重布线阈值

% 使用数组替代结构体
maxNodes = Maxiter + 1000; % 预留一些空间
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

% 使用单独的节点计数器
node_count = 1;

% 进度显示
best_length = inf;
Goal_index = 0;
findpath = 0;

% 统计信息
rewire_count = 0;
parent_select_count = 0;

% 可视化更新参数
visual_update_interval = 500;
last_visual_update = 0;

for iter = 1:Maxiter
    if node_count > 15000
        break
    end
    
    % 更新进度显示
    if mod(iter, 1000) == 0
        subplot(2,3,3);
        progress_str = sprintf('IRRT*进度: %d/%d\n节点数: %d\n当前最优: %.2f', ...
            iter, Maxiter, node_count, best_length);
        set(progress_text, 'String', progress_str);
        
        subplot(2,3,6);
        stats_str = sprintf('迭代次数: %d/%d\n节点数: %d\n重选父节点: %d\n重布线: %d', ...
            iter, Maxiter, node_count, parent_select_count, rewire_count);
        set(stats_text, 'String', stats_str);
        
        drawnow limitrate;
    end
    
    % 改进的采样策略
    if findpath
        % 椭球采样 - 只在找到路径后使用
        cb = max(Tv_cos(Goal_index)/2, 1);
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
    else
        % 未找到路径时的均匀采样
        randp_x = axisLWH(1)*rand(1)+axisStart(1);
        randp_y = axisLWH(2)*rand(1)+axisStart(2);
        randp_z = axisLWH(3)*rand(1)+axisStart(3);
    end
    
    
    % 寻找最近节点 - 使用线性搜索替代网格搜索
    min_dist_sq = inf;
    nearest_idx = 0;
    
    for idx = 1:node_count
        dx = randp_x - Tv_x(idx);
        dy = randp_y - Tv_y(idx);
        dz = randp_z - Tv_z(idx);
        dist_sq = dx*dx + dy*dy + dz*dz;
        
        if dist_sq < min_dist_sq
            min_dist_sq = dist_sq;
            nearest_idx = idx;
        end
    end
    
    if nearest_idx == 0
        continue;
    end
    
    r = sqrt(min_dist_sq);
    
    % 生成新节点
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
    
    % 边界检查
    if new_x < axisStart(1) || new_x > axisLWH(1) || ...
       new_y < axisStart(2) || new_y > axisLWH(2) || ...
       new_z < axisStart(3) || new_z > axisLWH(3)
        continue;
    end
    
    % 完整的碰撞检测
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
    
    %% 完整的IRRT*节点扩展过程
    new_x_candidate = new_x;
    new_y_candidate = new_y; 
    new_z_candidate = new_z;
    
    % 步骤1: 在附近节点中寻找更优的父节点
    best_parent = nearest_idx;
    best_cost = Tv_cos(nearest_idx) + step;
    min_distance = step;

    % 搜索新节点附近的潜在父节点 - 使用线性搜索
    candidate_parents = [];
    
    for idx = 1:node_count
        if idx ~= nearest_idx
            % 计算距离
            dx_candidate = new_x_candidate - Tv_x(idx);
            dy_candidate = new_y_candidate - Tv_y(idx);
            dz_candidate = new_z_candidate - Tv_z(idx);
            dist_candidate = sqrt(dx_candidate*dx_candidate + dy_candidate*dy_candidate + dz_candidate*dz_candidate);
            
            if dist_candidate <= rewire_Thr
                candidate_parents(end+1) = idx;
            end
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
    x_new_near = [];
    
    % 使用线性搜索寻找附近节点
    for idx = 1:node_count
        if idx ~= node_count && idx ~= best_parent
            % 计算距离
            dx = Tv_x(idx) - new_x_candidate;
            dy = Tv_y(idx) - new_y_candidate;
            dz = Tv_z(idx) - new_z_candidate;
            dist = sqrt(dx*dx + dy*dy + dz*dz);
            if dist <= rewire_Thr
                x_new_near(end+1) = idx;
            end
        end
    end

    % 对每个附近节点进行重布线
    for k = 1:length(x_new_near)
        candidate_idx = x_new_near(k);
        
        % 计算从新节点到候选节点的代价
        dx = Tv_x(candidate_idx) - new_x_candidate;
        dy = Tv_y(candidate_idx) - new_y_candidate;
        dz = Tv_z(candidate_idx) - new_z_candidate;
        dist = sqrt(dx*dx + dy*dy + dz*dz);
        
        new_cost = Tv_cos(node_count) + dist;
        
        % 如果新代价更小，则检查路径是否无碰撞
        if new_cost < Tv_cos(candidate_idx)
            % 碰撞检测：从新节点到候选节点
            startp_check.x = new_x_candidate;
            startp_check.y = new_y_candidate;
            startp_check.z = new_z_candidate;
            endp_check.x = Tv_x(candidate_idx);
            endp_check.y = Tv_y(candidate_idx);
            endp_check.z = Tv_z(candidate_idx);
            
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
                old_parent = Tv_pre(candidate_idx);
                old_cost_val = Tv_cos(candidate_idx);
                
                Tv_pre(candidate_idx) = node_count;
                Tv_cos(candidate_idx) = new_cost;
                
                % 内联代价传播逻辑
                cost_diff = new_cost - old_cost_val;
                queue = candidate_idx;
                
                while ~isempty(queue)
                    current_idx = queue(1);
                    queue(1) = [];
                    
                    % 找到所有直接子节点
                    children = find(Tv_pre == current_idx);
                    
                    % 更新所有子节点的代价
                    for child_idx = children'
                        if child_idx <= node_count
                            Tv_cos(child_idx) = Tv_cos(child_idx) + cost_diff;
                            queue(end+1) = child_idx;
                        end
                    end
                end
                
                rewire_count = rewire_count + 1;
            end
        end
    end 
    
    % 检查是否接近目标并更新最佳路径
    dist_to_goal = norm([new_x_candidate, new_y_candidate, new_z_candidate] - goalPoint);
    
    if dist_to_goal < Thr
        if ~findpath % 第一次找到路径
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
            best_length = Tv_cos(node_count);
            fprintf('首次找到路径! 长度: %.2f\n', best_length);
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
                fprintf('找到更好路径! 实际长度: %.2f\n', best_length);
            end
        end
    end
    
    % 定期更新可视化
    if iter - last_visual_update >= visual_update_interval || iter <= 1000
        update_visualization(Tv_x, Tv_y, Tv_z, Tv_pre, node_count, rrt_tree_plot, rrt_nodes_plot);
        last_visual_update = iter;
    end
end

%% 提取最终路径
if Goal_index > 0 && Goal_index <= node_count
    path_indices = [];
    current_idx = Goal_index;
    
    while current_idx > 0   
        path_indices = [current_idx, path_indices];
        current_idx = Tv_pre(current_idx);
        % 防止无限循环
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
else
    final_path_length = inf;
    fprintf('未找到可行路径！\n');
end
a = final_path_length;
t = toc;
%% 最终可视化
update_final_visualization(Tv_x, Tv_y, Tv_z, path_indices, final_path_plot, ...
    best_length, t, Maxiter, node_count, parent_select_count, rewire_count);



fprintf('\n=== IRRT*路径规划完成 ===\n');
fprintf('最终路径长度: %.2f\n', final_path_length);
fprintf('总运行时间: %.2f秒\n', t);
fprintf('总迭代次数: %d\n', Maxiter);
fprintf('总节点数: %d\n', node_count);
fprintf('重选父节点成功次数: %d\n', parent_select_count);
fprintf('重布线成功次数: %d\n', rewire_count);

end

%% 辅助函数：更新可视化
function update_visualization(Tv_x, Tv_y, Tv_z, Tv_pre, node_count, tree_plot, nodes_plot)
    % 只显示最近1000个节点以提高性能
    start_idx = max(1, node_count - 1000);
    
    % 构建树结构数据
    tree_x = [];
    tree_y = [];
    tree_z = [];
    
    for i = start_idx:node_count
        if Tv_pre(i) > 0 && Tv_pre(i) <= node_count
            tree_x = [tree_x, Tv_x(Tv_pre(i)), Tv_x(i), NaN];
            tree_y = [tree_y, Tv_y(Tv_pre(i)), Tv_y(i), NaN];
            tree_z = [tree_z, Tv_z(Tv_pre(i)), Tv_z(i), NaN];
        end
    end
    
    set(tree_plot, 'XData', tree_x, 'YData', tree_y, 'ZData', tree_z);
    set(nodes_plot, 'XData', Tv_x(1:node_count), 'YData', Tv_y(1:node_count), 'ZData', Tv_z(1:node_count));
    
    drawnow limitrate;
end

%% 辅助函数：最终可视化
function update_final_visualization(Tv_x, Tv_y, Tv_z, path_indices, final_path_plot, ...
    final_path_length, t, Maxiter, node_count, parent_select_count, rewire_count)
    
    % 绘制最终路径
    if ~isempty(path_indices)
        set(final_path_plot, 'XData', Tv_x(path_indices), 'YData', Tv_y(path_indices), 'ZData', Tv_z(path_indices));
        
        % 在路径节点上添加标记
        hold on;
        scatter3(Tv_x(path_indices), Tv_y(path_indices), Tv_z(path_indices), ...
                50, 'r', 'filled', 'MarkerEdgeColor', 'k');
    end
    
    % 更新统计信息
    subplot(2,3,3);
    progress_str = sprintf('IRRT*完成!\n最终路径长度: %.2f\n总时间: %.2f秒\n节点数: %d\n重选父节点: %d\n重布线: %d', ...
        final_path_length, t, node_count, parent_select_count, rewire_count);
    text(0.1, 0.5, progress_str, 'FontSize', 12, 'FontWeight', 'bold', 'VerticalAlignment', 'top');
    
    subplot(2,3,6);
    stats_str = sprintf('总迭代次数: %d\n总节点数: %d\n重选父节点: %d\n重布线: %d\n目标阈值: %.1f\n步长: %.1f', ...
        Maxiter, node_count, parent_select_count, rewire_count, 8, 6);
    text(0.1, 0.7, stats_str, 'FontSize', 10, 'VerticalAlignment', 'top');
    
    % 在主图添加完成标记
    subplot(2,3,[1,2,4,5]);
    if final_path_length < inf
        title(sprintf('三维路径规划 - IRRT*算法 (路径长度: %.2f)', final_path_length), ...
            'FontSize', 12, 'FontWeight', 'bold');
        text(100, 100, 160, sprintf('? 路径规划完成!\n路径长度: %.2f\n时间: %.2f秒', final_path_length, t), ...
            'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', ...
            'BackgroundColor', 'white', 'EdgeColor', 'black');
    else
        title('三维路径规划 - IRRT*算法 (未找到路径)', 'FontSize', 12, 'FontWeight', 'bold');
        text(100, 100, 160, '? 未找到可行路径', ...
            'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', ...
            'BackgroundColor', 'white', 'EdgeColor', 'black');
    end
    
    drawnow;
end