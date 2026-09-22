 function  [a,t2]=RRSg()
tic;
%% 定义地图
resolution = 1; % 网格长度

left_bound = 0;
right_bound = 300;
lower_bound = 0;
upper_bound = 300;   
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
square_obstacle(1,:) = [95,80,20,40]; % 方形障碍物1
square_obstacle(2,:) = [85,145,20,40]; % 方形障碍物2
square_obstacle(3,:) = [135,115,30,30]; %方形障碍物3
squareob = [square_obstacle; wall_obstacle];
for i=1:1:size(square_obstacle,1)
   h_square=fill([square_obstacle(i,1),square_obstacle(i,1)+square_obstacle(i,3),square_obstacle(i,1)+square_obstacle(i,3),square_obstacle(i,1)],...
         [square_obstacle(i,2),square_obstacle(i,2),square_obstacle(i,2)+square_obstacle(i,4),square_obstacle(i,2)+square_obstacle(i,4)],'k');
end

%绘制圆形障碍物
round_obstacle(1,:) = [175,175,12]; %圆形障碍物1
round_obstacle(2,:) = [205,175,12]; %圆形障碍物2
round_obstacle(3,:) = [145,185,12]; %圆形障碍物2
for i=1:1:size(round_obstacle,1)
    center = [round_obstacle(i,1),round_obstacle(i,2)]; % 圆心 (x0, y0)
    r = round_obstacle(i,3); % 半径
    theta = 0:0.01*pi:2*pi;
    x = r * cos(theta) + center(1);
    y = r * sin(theta) + center(2);
    h_round=fill(x, y, 'k', 'EdgeColor', 'k', 'LineWidth', 1.5);
end



% 参数初始化
x_I = 76; y_I = 76;           % 设置初始点
x_G = 220; y_G = 220;       % 设置目标点p
GoalThreshold = 2;         % 设置目标点阈值
Delta = 3;                 % 设置扩展步长 default:30
RadiusForNeib = 8;         % rewire的范围,半径r
MaxIterations = 10000;       % 最大迭代次数
UpdateTime = 50;            % 更新路径的时间间隔
DelayTime = 0.001;          % 绘图延迟时间

% 使用数组代替结构体 - 预分配内存
max_nodes = MaxIterations + 1000; % 预分配足够大的空间
T_x = zeros(1, max_nodes);
T_y = zeros(1, max_nodes);
T_xPrev = zeros(1, max_nodes);
T_yPrev = zeros(1, max_nodes);
T_totalCost = zeros(1, max_nodes);
T_indPrev = zeros(1, max_nodes);

% 建树初始化
T_x(1) = x_I;             % 把起始节点加入到T中
T_y(1) = y_I; 
T_xPrev(1) = x_I;         % 节点的父节点坐标:起点的父节点是其本身
T_yPrev(1) = y_I;
T_totalCost(1) = 0;       % 从起始节点开始的累计cost，这里取欧氏距离
T_indPrev(1) = 0;         % 父节点的索引

% 开始构建树
plot(x_I, y_I, 'mo', 'MarkerSize',10, 'MarkerFaceColor','m');   % 绘制起点和目标点
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
% Draw the goal circle
theta = linspace(0,2*pi);
goal_circle.x = GoalThreshold*cos(theta) + x_G;
goal_circle.y = GoalThreshold*sin(theta) + y_G;
plot(goal_circle.x,goal_circle.y,'--k','LineWidth',0.8*resolution);

count = 1;
pHandleList = [];
lHandleList = [];
resHandleList = [];
findPath = 0;
update_count = 0;
path_x = [];
path_y = [];
Goal_index = 0;
leng = zeros(1, MaxIterations);

for iter = 1:MaxIterations
    if iter > 1
        leng(iter) = leng(iter-1);
    else
        leng(iter) = 0;
    end
    if count>2000
        break
    end
    %Step 1: 在地图中随机采样一个点x_rand (Sample)
    x_rand = [unifrnd(left_bound,right_bound), unifrnd(lower_bound,upper_bound)]; % 产生随机点(x,y)
    
    %Step 2: 遍历树，从树中找到最近邻近点x_near (Near)
    minDis = sqrt((x_rand(1) - T_x(1))^2 + (x_rand(2) - T_y(1))^2);
    minIndex = 1;
    for i = 2:count
        distance = sqrt((x_rand(1) - T_x(i))^2 + (x_rand(2) - T_y(i))^2);   % 两节点间距离
        if distance < minDis
            minDis = distance;
            minIndex = i;   
        end     
    end
    
    x_near(1) = T_x(minIndex);    % 找到当前树中离x_rand最近的节点
    x_near(2) = T_y(minIndex);
    temp_parent = minIndex;         % 临时父节点的索引
    temp_cost = Delta + T_totalCost(minIndex);   % 临时累计代价

    %Step 3: 扩展得到x_new节点 (Steer)
    theta = atan2((x_rand(2) - x_near(2)), (x_rand(1) - x_near(1)));
    x_new(1) = x_near(1) + cos(theta) * Delta;
    x_new(2) = x_near(2) + sin(theta) * Delta;  
    
    % 检查节点是否是collision-free
    if ~collisionChecking(x_near, x_new, squareob, round_obstacle) 
        continue;   % 有障碍物
    end

    %Step 4: 在以x_new为圆心,半径为R的圆内搜索节点 (NearC)
    disToNewList = [];    % 每次循环要把队列清空
    nearIndexList = [];
    for index_near = 1:count
        disTonew = sqrt((x_new(1) - T_x(index_near))^2 + (x_new(2) - T_y(index_near))^2);
        if disTonew < RadiusForNeib    % 满足条件:欧氏距离小于R
            disToNewList = [disToNewList disTonew];     % 满足条件的所有节点到x_new的cost
            nearIndexList = [nearIndexList index_near];     % 满足条件的所有节点基于树T的索引
        end
    end
    
    %Step 5: 选择x_new的父节点,使x_new的累计cost最小 (ChooseParent)
    for cost_index = 1:length(nearIndexList)    % cost_index是基于disToNewList的索引,不是整棵树的索引
        costToNew = disToNewList(cost_index) + T_totalCost(nearIndexList(cost_index));
        if costToNew < temp_cost    % temp_cost为通过minDist节点的路径的cost
            x_mincost(1) = T_x(nearIndexList(cost_index));     % 符合剪枝条件节点的坐标
            x_mincost(2) = T_y(nearIndexList(cost_index));
            if ~collisionChecking(x_mincost, x_new, squareob, round_obstacle) 
                continue;   % 有障碍物
            end
            temp_cost = costToNew;
            temp_parent = nearIndexList(cost_index);
        end
    end
    
    %Step 6: 将x_new插入树T (AddNodeEdge)
    count = count + 1;    % 最新节点的索引
    
    T_x(count) = x_new(1);          
    T_y(count) = x_new(2); 
    T_xPrev(count) = T_x(temp_parent);     
    T_yPrev(count) = T_y(temp_parent);
    T_totalCost(count) = temp_cost; 
    T_indPrev(count) = temp_parent;     % 其父节点x_near的index
    
    l_handle = plot([T_xPrev(count), x_new(1)], [T_yPrev(count), x_new(2)], 'b', 'Linewidth', 0.01);
    p_handle = plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 0.01, 'MarkerFaceColor','k');
   
    pHandleList = [pHandleList p_handle];    % 绘图的句柄索引即为count
    lHandleList = [lHandleList l_handle];
    
    %Step 7: 剪枝 (rewire)
    for rewire_index = 1:length(nearIndexList)
        if nearIndexList(rewire_index) ~= temp_parent    % 若不是之前计算的最小cost的节点
            newCost = temp_cost + disToNewList(rewire_index);    % 计算neib经过x_new再到起点的代价          
            if newCost < T_totalCost(nearIndexList(rewire_index))    % 需要剪枝
                x_neib(1) = T_x(nearIndexList(rewire_index));     % 符合剪枝条件节点的坐标
                x_neib(2) = T_y(nearIndexList(rewire_index));
                if ~collisionChecking(x_neib, x_new, squareob, round_obstacle) 
                    continue;   % 有障碍物
                end
                 old_cost = T_totalCost(nearIndexList(rewire_index));
                T_xPrev(nearIndexList(rewire_index)) = x_new(1);      % 对该neighbor信息进行更新
                T_yPrev(nearIndexList(rewire_index)) = x_new(2);
                T_totalCost(nearIndexList(rewire_index)) = newCost;
                T_indPrev(nearIndexList(rewire_index)) = count;       % x_new的索引
                  % 使用find函数进行代价传播 - 高效方法
                cost_diff = newCost - old_cost;
                
                % 第一步：找到所有直接子节点
                current_node = nearIndexList(rewire_index);
                direct_children = find(T_indPrev == current_node);
                
                if ~isempty(direct_children)
                    % 第二步：递归找到所有后代节点
                    all_descendants = direct_children;
                    queue = direct_children;
                    
                    while ~isempty(queue)
                        current_child = queue(1);
                        queue(1) = [];
                        
                        % 找到当前孩子的子节点
                        grandchildren = find(T_indPrev == current_child);
                        if ~isempty(grandchildren)
                            all_descendants = [all_descendants, grandchildren];
                            queue = [queue, grandchildren];
                        end
                    end
                    
                    % 第三步：更新所有后代节点的代价
                    for desc_idx = all_descendants
                        T_totalCost(desc_idx) = T_totalCost(desc_idx) + cost_diff;
                    end
                end
                lHandleList(nearIndexList(rewire_index)) = plot([T_x(nearIndexList(rewire_index)), x_new(1)],...
                    [T_y(nearIndexList(rewire_index)), x_new(2)], 'b', 'Linewidth', 0.01);
            end
        end
    end
    
    %Step 8:检查是否到达目标点附近 
    disToGoal = sqrt((x_new(1) - x_G)^2 + (x_new(2) - y_G)^2);
    if disToGoal < GoalThreshold && ~findPath    % 找到目标点，此条件只进入一次
        findPath = 1;

        count = count + 1;    % 手动将Goal加入到树中
        Goal_index = count;
        T_x(count) = x_G;          
        T_y(count) = y_G; 
        T_xPrev(count) = x_new(1);     
        T_yPrev(count) = x_new(2);
        T_totalCost(count) = T_totalCost(count - 1) + disToGoal;
        T_indPrev(count) = count - 1;     % 其父节点x_near的index
    end
    
    if findPath == 1
        update_count = update_count + 1;
        if update_count == UpdateTime
            update_count = 0;
            j = 2;
            path_x(1) = x_G; 
            path_y(1) = y_G;
            pathIndex = T_indPrev(Goal_index);
            while 1     
                path_x(j) = T_x(pathIndex);
                path_y(j) = T_y(pathIndex);
                pathIndex = T_indPrev(pathIndex);    % 沿终点回溯到起点
                if pathIndex == 0
                    break
                end
                j = j + 1;
            end  
            
            for delete_index = 1:length(resHandleList)
                delete(resHandleList(delete_index));
            end
            for j = 2:length(path_x)
                res_handle = plot([path_x(j), path_x(j-1)], [path_y(j), path_y(j-1)], 'g', 'Linewidth', 2);
                resHandleList = [resHandleList res_handle];
            end
        end
    end  
    
    if Goal_index
        leng(iter) = T_totalCost(Goal_index);
    end
end

% 绘制最终路径
for delete_index = 1:length(resHandleList)
    delete(resHandleList(delete_index));
end
for j = 2:length(path_x)
    res_handle = plot([path_x(j), path_x(j-1)], [path_y(j), path_y(j-1)], 'g', 'Linewidth', 2);
    resHandleList = [resHandleList res_handle];
end       

% 绘制收敛关系图
m = 1:iter-1    ;
x = m;
leng = leng(1:iter-1);
y = leng;

figure(3)
title('RRT*');
plot(x,y)
disp('The path is found!');
t2 = toc;
a = leng(end);
b = t2;
   end