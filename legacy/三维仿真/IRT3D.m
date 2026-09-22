 function [a,t] = IRT3D()


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

%% 改进的IRRT*算法 - 使用完整的碰撞检测
% fprintf('开始IRRT*优化...\n');

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
startPoint = pathPoint(1,:);
goalPoint = pathPoint(2,:);
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

for iter = 1:Maxiter
    if node_count > 12000
        break
    end
    
%     if mod(iter, 1000) == 0
%         fprintf('进度: %d/%d, 节点数: %d\n', iter, Maxiter, node_count);
%     end
    
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
%             fprintf('首次找到路径! 长度: %.2f\n', best_length);
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
%     fprintf('未找到可行路径！\n');
end

a = final_path_length;
t = toc;
 end

% fprintf('\n=== IRRT*路径规划完成 ===\n');
% fprintf('最终路径长度: %.2f\n', final_path_length);
% fprintf('总运行时间: %.2f秒\n', t);
% fprintf('总迭代次数: %d\n', Maxiter);
% fprintf('总节点数: %d\n', node_count);
% fprintf('重选父节点成功次数: %d\n', parent_select_count);
% fprintf('重布线成功次数: %d\n', rewire_count);
%  end