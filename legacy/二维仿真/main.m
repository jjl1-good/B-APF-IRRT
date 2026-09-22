%% ============ 路径规划算法性能测试与分析 ============
% MATLAB版本：2016b
% 预设随机种子确保结果可重复
% 包含三种环境测试：y, g, n

clc;
clear all;
close all;

%% ============ 1. 预设随机种子 ============
% 设置随机数生成器种子以确保结果可重复
rng(20231202, 'twister');  % 使用固定种子，可以修改为任意整数
fprintf('随机种子已设置为: 20231202\n\n');

%% ============ 2. 初始化数据结构 ============
% 为三种环境（y, g, n）初始化数据结构
% BAIRRT算法
BAIRRTy = struct('l', [], 't', []);
BAIRRTg = struct('l', [], 't', []);
BAIRRTn = struct('l', [], 't', []);

% IRTT算法
IRTTy = struct('l', [], 't', []);
IRTTg = struct('l', [], 't', []);
IRTTn = struct('l', [], 't', []);

% RRSTA算法
RRSTAy = struct('l', [], 't', []);
RRSTAg = struct('l', [], 't', []);
RRSTAn = struct('l', [], 't', []);

%% ============ 3. 记录开始时间 ============
total_start_time = tic;
fprintf('开始实验...\n');
fprintf('============================================\n');

%% ============ 4. y环境测试 ============
fprintf('============ y环境测试 ============\n');
y_start_time = tic;

% BAIRRT算法在y环境
fprintf('\n--- BAIRRTy 算法测试 ---\n');
for u = 1:50
    fprintf('BAIRRTy 第%d次运行...\n', u);
    [BAIRRTy(u).l, BAIRRTy(u).t] = BAIRy();
    fprintf('BAIRRTy 第%d次运行: 路径长度=%.2f, 时间=%.2f秒\n', u, BAIRRTy(u).l, BAIRRTy(u).t);
    
    % 算法函数使用独立工作区，无需在此清理主脚本变量
    close all hidden
end

% IRTT算法在y环境
fprintf('\n--- IRTTy 算法测试 ---\n');
for u = 1:50
    fprintf('IRTTy 第%d次运行...\n', u);
    [IRTTy(u).l, IRTTy(u).t] = IRTy();
    fprintf('IRTTy 第%d次运行: 路径长度=%.2f, 时间=%.2f秒\n', u, IRTTy(u).l, IRTTy(u).t);
    
    % 算法函数使用独立工作区，无需在此清理主脚本变量
    close all hidden
end

% RRSTA算法在y环境
fprintf('\n--- RRSTAy 算法测试 ---\n');
for u = 1:50
    fprintf('RRSTAy 第%d次运行...\n', u);
    [RRSTAy(u).l, RRSTAy(u).t] = RRSy();
    fprintf('RRSTAy 第%d次运行: 路径长度=%.2f, 时间=%.2f秒\n', u, RRSTAy(u).l, RRSTAy(u).t);
    
    % 算法函数使用独立工作区，无需在此清理主脚本变量
    close all hidden
end

y_elapsed_time = toc(y_start_time);
fprintf('\ny环境测试完成，耗时: %.2f 秒\n', y_elapsed_time);

%% ============ 5. g环境测试 ============
fprintf('\n============ g环境测试 ============\n');
g_start_time = tic;

% BAIRRT算法在g环境
fprintf('\n--- BAIRRTg 算法测试 ---\n');
for u = 1:50
    fprintf('BAIRRTg 第%d次运行...\n', u);
    [BAIRRTg(u).l, BAIRRTg(u).t] = BAIRg();
    fprintf('BAIRRTg 第%d次运行: 路径长度=%.2f, 时间=%.2f秒\n', u, BAIRRTg(u).l, BAIRRTg(u).t);
    
    % 算法函数使用独立工作区，无需在此清理主脚本变量
    close all hidden
end

% IRTT算法在g环境
fprintf('\n--- IRTTg 算法测试 ---\n');
for u = 1:50
    fprintf('IRTTg 第%d次运行...\n', u);
    [IRTTg(u).l, IRTTg(u).t] = IRTg();
    fprintf('IRTTg 第%d次运行: 路径长度=%.2f, 时间=%.2f秒\n', u, IRTTg(u).l, IRTTg(u).t);
    
    % 算法函数使用独立工作区，无需在此清理主脚本变量
    close all hidden
end

% RRSTA算法在g环境
fprintf('\n--- RRSTAg 算法测试 ---\n');
for u = 1:50
    fprintf('RRSTAg 第%d次运行...\n', u);
    [RRSTAg(u).l, RRSTAg(u).t] = RRSg();
    fprintf('RRSTAg 第%d次运行: 路径长度=%.2f, 时间=%.2f秒\n', u, RRSTAg(u).l, RRSTAg(u).t);
    
    % 算法函数使用独立工作区，无需在此清理主脚本变量
    close all hidden
end

g_elapsed_time = toc(g_start_time);
fprintf('\ng环境测试完成，耗时: %.2f 秒\n', g_elapsed_time);

%% ============ 6. n环境测试 ============
fprintf('\n============ n环境测试 ============\n');
n_start_time = tic;

% 注意：这里应该是BAIRn()而不是BAIRg()，根据您的命名规范
fprintf('\n--- BAIRRTn 算法测试 ---\n');
for u = 1:50
    fprintf('BAIRRTn 第%d次运行...\n', u);
    [BAIRRTn(u).l, BAIRRTn(u).t] = BAIRn();  % 修正：改为BAIRn()
    fprintf('BAIRRTn 第%d次运行: 路径长度=%.2f, 时间=%.2f秒\n', u, BAIRRTn(u).l, BAIRRTn(u).t);
    
    % 算法函数使用独立工作区，无需在此清理主脚本变量
    close all hidden
end

% IRTT算法在n环境
fprintf('\n--- IRTTn 算法测试 ---\n');
for u = 1:50
    fprintf('IRTTn 第%d次运行...\n', u);
    [IRTTn(u).l, IRTTn(u).t] = IRTn();
    fprintf('IRTTn 第%d次运行: 路径长度=%.2f, 时间=%.2f秒\n', u, IRTTn(u).l, IRTTn(u).t);
    
    % 算法函数使用独立工作区，无需在此清理主脚本变量
    close all hidden
end

% RRSTA算法在n环境
fprintf('\n--- RRSTAn 算法测试 ---\n');
for u = 1:50
    fprintf('RRSTAn 第%d次运行...\n', u);
    [RRSTAn(u).l, RRSTAn(u).t] = RRSn();
    fprintf('RRSTAn 第%d次运行: 路径长度=%.2f, 时间=%.2f秒\n', u, RRSTAn(u).l, RRSTAn(u).t);
    
    % 算法函数使用独立工作区，无需在此清理主脚本变量
    close all hidden
end

n_elapsed_time = toc(n_start_time);
fprintf('\nn环境测试完成，耗时: %.2f 秒\n', n_elapsed_time);

%% ============ 7. 数据提取和整理 ============
fprintf('\n============ 数据提取和整理 ============\n');

% 从结构体中提取数据到数组
% y环境
bairrt_y_lengths = [BAIRRTy.l];
bairrt_y_times = [BAIRRTy.t];
irtt_y_lengths = [IRTTy.l];
irtt_y_times = [IRTTy.t];
rrsta_y_lengths = [RRSTAy.l];
rrsta_y_times = [RRSTAy.t];

% g环境
bairrt_g_lengths = [BAIRRTg.l];
bairrt_g_times = [BAIRRTg.t];
irtt_g_lengths = [IRTTg.l];
irtt_g_times = [IRTTg.t];
rrsta_g_lengths = [RRSTAg.l];
rrsta_g_times = [RRSTAg.t];

% n环境
bairrt_n_lengths = [BAIRRTn.l];
bairrt_n_times = [BAIRRTn.t];
irtt_n_lengths = [IRTTn.l];
irtt_n_times = [IRTTn.t];
rrsta_n_lengths = [RRSTAn.l];
rrsta_n_times = [RRSTAn.t];

% 保存原始数据到mat文件
save('path_planning_raw_data.mat', ...
    'bairrt_y_lengths', 'bairrt_y_times', ...
    'irtt_y_lengths', 'irtt_y_times', ...
    'rrsta_y_lengths', 'rrsta_y_times', ...
    'bairrt_g_lengths', 'bairrt_g_times', ...
    'irtt_g_lengths', 'irtt_g_times', ...
    'rrsta_g_lengths', 'rrsta_g_times', ...
    'bairrt_n_lengths', 'bairrt_n_times', ...
    'irtt_n_lengths', 'irtt_n_times', ...
    'rrsta_n_lengths', 'rrsta_n_times');

fprintf('原始数据已保存到: path_planning_raw_data.mat\n');

%% ============ 8. 统计分析 ============
fprintf('\n============ 统计分析 ============\n');

% 创建结果表格（预先定义变量和9行空间，兼容 MATLAB R2016b）
results_table = table(cell(9,1), cell(9,1), nan(9,1), nan(9,1), ...
    nan(9,1), nan(9,1), nan(9,1), nan(9,1), nan(9,1), ...
    nan(9,1), nan(9,1), ...
    'VariableNames', {'Algorithm', 'Environment', 'SuccessRate', ...
    'AvgPathLength', 'StdPathLength', 'MinPathLength', 'MaxPathLength', ...
    'AvgTime', 'StdTime', 'SuccessCount', 'TotalCount'});

% 定义算法和环境名称
algorithms = {'BAIRRT'; 'IRTT'; 'RRSTA'};
environments = {'y'; 'g'; 'n'};

% 初始化计数器
row_idx = 1;

% 对每种算法和每个环境进行统计
for algo_idx = 1:length(algorithms)
    for env_idx = 1:length(environments)
        % 获取当前算法和环境的数据
        algo = algorithms{algo_idx};
        env = environments{env_idx};
        
        % 根据算法和环境选择数据
        switch algo
            case 'BAIRRT'
                switch env
                    case 'y'
                        lengths = bairrt_y_lengths;
                        times = bairrt_y_times;
                    case 'g'
                        lengths = bairrt_g_lengths;
                        times = bairrt_g_times;
                    case 'n'
                        lengths = bairrt_n_lengths;
                        times = bairrt_n_times;
                end
            case 'IRTT'
                switch env
                    case 'y'
                        lengths = irtt_y_lengths;
                        times = irtt_y_times;
                    case 'g'
                        lengths = irtt_g_lengths;
                        times = irtt_g_times;
                    case 'n'
                        lengths = irtt_n_lengths;
                        times = irtt_n_times;
                end
            case 'RRSTA'
                switch env
                    case 'y'
                        lengths = rrsta_y_lengths;
                        times = rrsta_y_times;
                    case 'g'
                        lengths = rrsta_g_lengths;
                        times = rrsta_g_times;
                    case 'n'
                        lengths = rrsta_n_lengths;
                        times = rrsta_n_times;
                end
        end
        
        % 计算统计指标
        % 成功率（路径长度非0的比例）
        success_mask = lengths ~= 0;
        success_rate = 100 * sum(success_mask) / length(lengths);
        
        % 有效数据（成功的数据）
        valid_lengths = lengths(success_mask);
        valid_times = times(success_mask);
        
        % 如果有效数据为空，设置默认值
        if isempty(valid_lengths)
            avg_length = NaN;
            std_length = NaN;
            min_length = NaN;
            max_length = NaN;
            avg_time = NaN;
            std_time = NaN;
        else
            avg_length = mean(valid_lengths);
            std_length = std(valid_lengths);
            min_length = min(valid_lengths);
            max_length = max(valid_lengths);
            avg_time = mean(valid_times);
            std_time = std(valid_times);
        end
        
        % 添加到结果表格
        results_table.Algorithm{row_idx} = algo;
        results_table.Environment{row_idx} = env;
        results_table.SuccessRate(row_idx) = success_rate;
        results_table.AvgPathLength(row_idx) = avg_length;
        results_table.StdPathLength(row_idx) = std_length;
        results_table.MinPathLength(row_idx) = min_length;
        results_table.MaxPathLength(row_idx) = max_length;
        results_table.AvgTime(row_idx) = avg_time;
        results_table.StdTime(row_idx) = std_time;
        results_table.SuccessCount(row_idx) = sum(success_mask);
        results_table.TotalCount(row_idx) = length(lengths);
        
        row_idx = row_idx + 1;
    end
end

%% ============ 9. 结果显示和保存 ============
% 显示结果表格
fprintf('\n============ 详细结果 ============\n');
disp(results_table);

% 保存结果到CSV文件
writetable(results_table, 'path_planning_results.csv');
fprintf('\n结果已保存到: path_planning_results.csv\n');

% 创建汇总表格（按算法）
fprintf('\n============ 按算法汇总 ============\n');
for algo_idx = 1:length(algorithms)
    algo = algorithms{algo_idx};
    algo_rows = strcmp(results_table.Algorithm, algo);
    algo_data = results_table(algo_rows, :);
    
    fprintf('\n--- %s 算法汇总 ---\n', algo);
    for i = 1:size(algo_data, 1)
        row = algo_data(i, :);
        fprintf('环境 %s: 成功率=%.1f%%, 平均路径长度=%.2f, 平均时间=%.2fs\n', ...
            row.Environment{1}, row.SuccessRate, row.AvgPathLength, row.AvgTime);
    end
end

% 创建汇总表格（按环境）
fprintf('\n============ 按环境汇总 ============\n');
for env_idx = 1:length(environments)
    env = environments{env_idx};
    env_rows = strcmp(results_table.Environment, env);
    env_data = results_table(env_rows, :);
    
    fprintf('\n--- %s 环境汇总 ---\n', env);
    for i = 1:size(env_data, 1)
        row = env_data(i, :);
        fprintf('算法 %s: 成功率=%.1f%%, 平均路径长度=%.2f, 平均时间=%.2fs\n', ...
            row.Algorithm{1}, row.SuccessRate, row.AvgPathLength, row.AvgTime);
    end
end

%% ============ 10. 可视化分析 ============
fprintf('\n============ 生成可视化图表 ============\n');

% 创建图形窗口
figure('Position', [100, 100, 1200, 800]);

% 子图1：成功率比较
subplot(2, 3, 1);
hold on;
colors = {'r', 'g', 'b'};
for algo_idx = 1:length(algorithms)
    algo = algorithms{algo_idx};
    algo_rows = strcmp(results_table.Algorithm, algo);
    success_rates = results_table.SuccessRate(algo_rows);
    
    bar_position = algo_idx:length(algorithms):(length(algorithms)*length(environments));
    bar(bar_position, success_rates, 'FaceColor', colors{algo_idx});
end
title('算法成功率比较');
ylabel('成功率 (%)');
set(gca, 'XTick', 1:length(environments)*length(algorithms));
xticklabels = {};
for env_idx = 1:length(environments)
    for algo_idx = 1:length(algorithms)
        xticklabels{end+1} = sprintf('%s-%s', environments{env_idx}, algorithms{algo_idx});
    end
end
set(gca, 'XTickLabel', xticklabels, 'XTickLabelRotation', 45);
legend(algorithms, 'Location', 'best');
grid on;
hold off;

% 子图2：平均路径长度比较
subplot(2, 3, 2);
hold on;
for algo_idx = 1:length(algorithms)
    algo = algorithms{algo_idx};
    algo_rows = strcmp(results_table.Algorithm, algo);
    avg_lengths = results_table.AvgPathLength(algo_rows);
    
    bar_position = algo_idx:length(algorithms):(length(algorithms)*length(environments));
    bar(bar_position, avg_lengths, 'FaceColor', colors{algo_idx});
end
title('平均路径长度比较');
ylabel('路径长度');
set(gca, 'XTick', 1:length(environments)*length(algorithms));
set(gca, 'XTickLabel', xticklabels, 'XTickLabelRotation', 45);
legend(algorithms, 'Location', 'best');
grid on;
hold off;

% 子图3：平均运行时间比较
subplot(2, 3, 3);
hold on;
for algo_idx = 1:length(algorithms)
    algo = algorithms{algo_idx};
    algo_rows = strcmp(results_table.Algorithm, algo);
    avg_times = results_table.AvgTime(algo_rows);
    
    bar_position = algo_idx:length(algorithms):(length(algorithms)*length(environments));
    bar(bar_position, avg_times, 'FaceColor', colors{algo_idx});
end
title('平均运行时间比较');
ylabel('运行时间 (秒)');
set(gca, 'XTick', 1:length(environments)*length(algorithms));
set(gca, 'XTickLabel', xticklabels, 'XTickLabelRotation', 45);
legend(algorithms, 'Location', 'best');
grid on;
hold off;

% 子图4：y环境路径长度分布
subplot(2, 3, 4);
hold on;
for algo_idx = 1:length(algorithms)
    algo = algorithms{algo_idx};
    switch algo
        case 'BAIRRT'
            data = bairrt_y_lengths(bairrt_y_lengths ~= 0);
        case 'IRTT'
            data = irtt_y_lengths(irtt_y_lengths ~= 0);
        case 'RRSTA'
            data = rrsta_y_lengths(rrsta_y_lengths ~= 0);
    end
    
    if ~isempty(data)
        histogram(data, 'FaceColor', colors{algo_idx}, 'EdgeColor', 'k', 'DisplayName', algo);
    end
end
title('y环境路径长度分布');
xlabel('路径长度');
ylabel('频数');
legend('Location', 'best');
grid on;
hold off;

% 子图5：g环境路径长度分布
subplot(2, 3, 5);
hold on;
for algo_idx = 1:length(algorithms)
    algo = algorithms{algo_idx};
    switch algo
        case 'BAIRRT'
            data = bairrt_g_lengths(bairrt_g_lengths ~= 0);
        case 'IRTT'
            data = irtt_g_lengths(irtt_g_lengths ~= 0);
        case 'RRSTA'
            data = rrsta_g_lengths(rrsta_g_lengths ~= 0);
    end
    
    if ~isempty(data)
        histogram(data, 'FaceColor', colors{algo_idx}, 'EdgeColor', 'k', 'DisplayName', algo);
    end
end
title('g环境路径长度分布');
xlabel('路径长度');
ylabel('频数');
legend('Location', 'best');
grid on;
hold off;

% 子图6：n环境路径长度分布
subplot(2, 3, 6);
hold on;
for algo_idx = 1:length(algorithms)
    algo = algorithms{algo_idx};
    switch algo
        case 'BAIRRT'
            data = bairrt_n_lengths(bairrt_n_lengths ~= 0);
        case 'IRTT'
            data = irtt_n_lengths(irtt_n_lengths ~= 0);
        case 'RRSTA'
            data = rrsta_n_lengths(rrsta_n_lengths ~= 0);
    end
    
    if ~isempty(data)
        histogram(data, 'FaceColor', colors{algo_idx}, 'EdgeColor', 'k', 'DisplayName', algo);
    end
end
title('n环境路径长度分布');
xlabel('路径长度');
ylabel('频数');
legend('Location', 'best');
grid on;
hold off;

% 保存图表
saveas(gcf, 'path_planning_analysis.png');
fprintf('图表已保存为: path_planning_analysis.png\n');

%% ============ 11. 实验总结 ============
total_elapsed_time = toc(total_start_time);
fprintf('\n============ 实验总结 ============\n');
fprintf('总实验次数: %d次\n', 50*3*3);
fprintf('总耗时: %.2f 秒 (约 %.2f 分钟)\n', total_elapsed_time, total_elapsed_time/60);
fprintf('随机种子: 20231202\n');
fprintf('数据文件: path_planning_raw_data.mat\n');
fprintf('结果文件: path_planning_results.csv\n');
fprintf('图表文件: path_planning_analysis.png\n');
fprintf('\n实验完成！\n');

%% ============ 12. 函数定义（如果需要在同一文件中） ============
% 注意：这里只是函数框架，实际函数需要在单独文件中定义
% function [length, time] = BAIRy()
%     % BAIRRT算法在y环境的实现
%     % 返回路径长度和运行时间
% end
%
% function [length, time] = IRTy()
%     % IRTT算法在y环境的实现
%     % 返回路径长度和运行时间
% end
%
% function [length, time] = RRSy()
%     % RRSTA算法在y环境的实现
%     % 返回路径长度和运行时间
% end
%
% % 其他函数类似定义...
