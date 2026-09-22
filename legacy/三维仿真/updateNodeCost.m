%% 修复的代价传播函数
function updateNodeCost(node_idx, old_cost, new_cost, Tv_cos, Tv_pre)
    % 更新节点及其所有后代的代价
    queue = node_idx;
    cost_diff = new_cost - old_cost;
    
    while ~isempty(queue)
        current_idx = queue(1);
        queue(1) = [];
        
        % 更新当前节点的代价
        Tv_cos(current_idx) = Tv_cos(current_idx) + cost_diff;
        
        % 找到所有直接子节点
        children = find(Tv_pre == current_idx);
        
        % 修复：安全处理空数组和维度
        if ~isempty(children)
            % 确保children是行向量
            children = reshape(children, 1, []); % 强制转换为行向量
            queue = [queue, children];
        end
    end
end
