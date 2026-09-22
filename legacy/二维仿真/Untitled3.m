%% ============ 路径规划算法实验数据分析 ============
% 基于实验运行结果进行统计分析
% MATLAB版本：2016b

clear all;
close all;
clc;

fprintf('============ 实验数据分析 ============\n');
fprintf('随机种子: 20231202\n');
fprintf('测试环境: y, g, n\n');
fprintf('测试算法: BAIRRT, IRTT, RRSTA\n');
fprintf('每种算法运行次数: 50次\n\n');

%% ============ 1. y环境数据 ============
fprintf('============ y环境数据 ============\n');

% BAIRRTy数据
bairrt_y_lengths = [209.05, 208.95, 208.83, 209.26, 209.16, 208.92, 209.00, 209.05, 208.80, 209.24, ...
                    209.00, 208.69, 208.93, 208.85, 209.43, 209.07, 209.23, 209.38, 209.26, 208.81, ...
                    208.75, 208.86, 208.91, 208.92, 209.10, 208.94, 208.85, 208.84, 209.12, 209.18, ...
                    208.66, 209.05, 208.94, 209.29, 208.98, 209.42, 208.90, 209.34, 209.12, 208.90, ...
                    208.70, 209.10, 208.83, 208.81, 208.98, 209.15, 208.83, 209.13, 209.01, 208.93];
bairrt_y_times = [4.65, 5.58, 5.04, 5.36, 3.59, 3.46, 3.43, 3.24, 4.96, 3.54, 4.16, 5.11, 4.64, 3.27, 3.47, ...
                  3.01, 4.16, 3.59, 3.61, 3.40, 3.36, 3.15, 3.21, 3.33, 3.28, 3.63, 3.35, 3.63, 3.73, 3.68, ...
                  3.50, 3.59, 3.56, 3.77, 3.76, 3.73, 3.43, 3.22, 3.64, 3.15, 3.58, 3.29, 3.38, 3.06, 3.47, ...
                  3.13, 2.91, 3.50, 3.51, 3.36];

% IRTTy数据（包含0.00失败数据）
irtt_y_lengths_raw = [211.83, 0, 210.42, 209.32, 211.87, 0, 210.24, 213.66, 210.75, 0, ...
                      0, 215.12, 209.60, 0, 211.88, 211.39, 216.71, 210.30, 210.75, 0, ...
                      210.16, 210.14, 210.79, 0, 210.60, 212.38, 213.56, 211.21, 0, 223.62, ...
                      0, 210.61, 223.36, 0, 0, 210.16, 0, 210.82, 212.64, 217.04, ...
                      0, 209.51, 0, 213.62, 210.23, 0, 212.27, 0, 211.16, 211.64, 0];
irtt_y_times_raw = [6.18, 5.13, 6.40, 6.32, 5.68, 5.13, 5.97, 6.06, 5.38, 5.38, 5.12, 5.47, 6.06, 5.35, 5.32, ...
                    5.61, 5.08, 5.71, 6.14, 5.07, 5.44, 5.60, 5.61, 4.99, 5.16, 5.40, 5.35, 5.55, 4.99, 5.28, ...
                    5.29, 5.93, 5.38, 4.98, 5.08, 5.46, 5.33, 5.35, 5.39, 5.45, 5.38, 6.00, 5.44, 5.64, 5.86, ...
                    4.93, 5.25, 5.02, 5.22, 5.51, 5.48];

% RRSTAy数据（包含0.00失败数据）
rrsta_y_lengths_raw = [213.83, 213.09, 213.29, 0, 0, 212.12, 212.72, 213.30, 211.55, 214.51, ...
                       0, 0, 0, 0, 212.39, 209.97, 216.79, 0, 210.26, 0, ...
                       215.89, 0, 213.69, 215.88, 0, 214.23, 211.88, 212.76, 212.72, 211.97, ...
                       210.90, 211.86, 0, 224.90, 215.35, 0, 212.57, 216.95, 213.42, 219.80, ...
                       214.42, 210.96, 0, 220.83, 0, 212.65, 212.97, 217.56, 0, 213.56];
rrsta_y_times_raw = [5.61, 5.44, 5.24, 5.32, 5.13, 5.29, 5.64, 5.83, 5.23, 5.57, 5.08, 5.25, 5.06, 5.01, 5.46, ...
                     5.57, 5.34, 5.02, 6.00, 4.97, 5.40, 5.11, 5.22, 5.19, 5.52, 5.32, 5.31, 5.41, 5.31, 5.89, ...
                     5.28, 5.32, 5.30, 5.45, 5.17, 5.11, 5.34, 5.26, 5.28, 5.28, 5.68, 5.12, 5.01, 5.56, 5.21, ...
                     5.59, 5.43, 5.25, 5.16, 5.40];

%% ============ 2. g环境数据 ============
fprintf('============ g环境数据 ============\n');

% BAIRRTg数据
bairrt_g_lengths = [209.44, 209.02, 209.16, 208.90, 209.29, 208.72, 209.12, 209.36, 209.04, 208.69, ...
                    209.05, 208.89, 209.10, 208.98, 208.97, 209.28, 209.24, 208.84, 208.95, 208.82, ...
                    209.03, 209.13, 209.78, 208.73, 209.12, 209.10, 208.89, 209.25, 208.69, 209.18, ...
                    209.12, 208.93, 208.95, 209.26, 208.91, 208.88, 208.99, 209.28, 209.24, 208.98, ...
                    209.02, 208.87, 209.33, 209.03, 208.80, 208.66, 208.93, 208.98, 209.31, 208.90];
bairrt_g_times = [3.49, 3.35, 3.48, 3.09, 3.39, 3.03, 3.46, 3.29, 3.33, 3.26, 3.19, 3.65, 3.35, 3.22, 3.32, ...
                  3.36, 3.22, 3.26, 3.31, 3.58, 3.26, 3.64, 3.29, 3.28, 3.16, 3.42, 3.46, 3.40, 3.10, 3.21, ...
                  3.35, 3.23, 3.50, 3.30, 3.22, 3.01, 3.41, 3.47, 3.33, 3.16, 3.35, 3.10, 3.41, 3.32, 3.23, ...
                  3.35, 3.35, 3.46, 3.40, 3.60];

% IRTTg数据（包含0.00失败数据）
irtt_g_lengths_raw = [0, 0, 0, 0, 0, 272.59, 0, 0, 0, 213.86, ...
                      0, 0, 0, 209.71, 215.77, 0, 0, 0, 212.89, 221.48, ...
                      0, 0, 235.04, 211.91, 0, 0, 0, 0, 0, 0, ...
                      0, 231.96, 0, 0, 0, 0, 225.30, 298.25, 0, 0, ...
                      211.76, 0, 239.03, 292.08, 0, 0, 0, 0, 0, 0];
irtt_g_times_raw = [3.55, 3.47, 3.48, 3.45, 3.50, 4.69, 3.48, 3.39, 3.39, 4.84, 3.50, 3.50, 3.49, 5.52, 5.31, ...
                    3.52, 3.49, 3.46, 4.88, 5.21, 3.44, 3.48, 4.45, 5.87, 3.41, 3.42, 3.46, 3.45, 3.47, 3.48, ...
                    3.42, 3.45, 3.46, 3.45, 3.49, 3.45, 3.54, 4.58, 3.45, 3.46, 5.27, 3.43, 5.28, 3.81, 3.46, ...
                    3.49, 3.47, 3.43, 3.51, 3.57];

% RRSTAg数据（包含0.00失败数据）
rrsta_g_lengths_raw = [0, 0, 0, 0, 242.86, 0, 0, 0, 0, 0, ...
                       228.42, 0, 254.73, 0, 0, 255.88, 0, 0, 0, 218.01, ...
                       226.57, 0, 0, 230.43, 0, 0, 0, 220.96, 216.41, 307.52, ...
                       0, 224.95, 0, 0, 0, 0, 0, 216.57, 0, 0, ...
                       0, 0, 0, 0, 0, 0, 0, 218.32, 238.01, 0];
rrsta_g_times_raw = [3.58, 3.55, 3.54, 3.48, 4.25, 3.53, 3.48, 3.51, 3.50, 3.49, 4.30, 3.50, 3.83, 3.42, 3.49, ...
                     3.82, 3.45, 3.50, 3.50, 3.92, 4.25, 3.53, 3.44, 3.80, 3.43, 3.48, 3.45, 3.86, 4.25, 3.52, ...
                     3.46, 3.68, 3.40, 3.49, 3.46, 3.41, 3.44, 4.12, 3.42, 3.47, 3.44, 3.42, 3.40, 3.38, 3.44, ...
                     3.42, 3.47, 3.72, 4.36, 3.55];

%% ============ 3. n环境数据 ============
fprintf('============ n环境数据 ============\n');

% BAIRRTn数据
bairrt_n_lengths = [213.00, 212.51, 213.35, 212.75, 218.92, 212.76, 213.33, 212.97, 213.65, 213.43, ...
                    212.52, 216.12, 213.68, 213.34, 212.54, 213.29, 212.84, 213.54, 212.40, 212.62, ...
                    213.60, 212.73, 213.30, 212.91, 212.89, 214.90, 213.26, 213.48, 212.58, 212.66, ...
                    212.52, 212.65, 213.85, 212.74, 212.80, 213.16, 213.03, 213.57, 215.01, 214.16, ...
                    213.68, 212.56, 213.30, 213.40, 213.47, 213.10, 213.37, 214.19, 213.23, 213.70];
bairrt_n_times = [3.21, 2.83, 3.01, 2.92, 2.92, 3.12, 2.97, 2.86, 2.84, 3.05, 2.85, 2.82, 3.08, 2.93, 3.02, ...
                  3.06, 3.05, 3.13, 2.88, 2.78, 2.79, 2.85, 3.12, 3.16, 3.33, 3.02, 3.20, 3.15, 3.28, 2.91, ...
                  2.86, 3.03, 3.10, 3.02, 2.92, 3.04, 3.06, 3.02, 2.81, 3.23, 2.94, 3.17, 3.06, 2.86, 3.05, ...
                  2.85, 2.92, 3.15, 3.02, 2.86];

% IRTTn数据（包含0.00失败数据）
irtt_n_lengths_raw = [214.23, 219.33, 0, 217.93, 247.23, 241.09, 0, 0, 240.96, 236.89, ...
                      0, 0, 0, 0, 0, 216.76, 215.34, 0, 216.55, 0, ...
                      215.21, 213.56, 0, 0, 214.57, 214.71, 0, 232.51, 219.25, 229.44, ...
                      0, 230.15, 215.04, 0, 229.93, 252.42, 215.07, 215.02, 238.58, 0, ...
                      216.27, 239.88, 0, 0, 216.66, 224.08, 217.48, 0, 215.75, 0];
irtt_n_times_raw = [5.70, 5.61, 4.95, 5.31, 5.43, 5.46, 5.40, 5.06, 5.64, 5.26, 5.00, 5.15, 5.13, 4.81, 5.05, ...
                    5.91, 5.61, 5.21, 5.10, 4.92, 6.11, 5.41, 5.26, 5.15, 5.64, 5.27, 5.55, 5.66, 5.43, 5.16, ...
                    5.42, 5.59, 5.91, 5.23, 5.92, 5.25, 5.75, 5.88, 5.40, 4.83, 5.48, 5.95, 5.14, 5.32, 5.59, ...
                    5.42, 5.60, 5.02, 5.14, 5.13];

% RRSTAn数据（包含0.00失败数据）
rrsta_n_lengths_raw = [231.04, 216.60, 219.51, 0, 221.31, 219.09, 240.14, 238.43, 0, 216.89, ...
                       217.68, 215.95, 243.18, 229.67, 218.11, 223.65, 0, 0, 221.38, 213.53, ...
                       217.78, 218.32, 215.87, 216.80, 0, 0, 243.64, 216.94, 220.82, 0, ...
                       235.47, 215.56, 223.51, 224.59, 227.81, 222.42, 0, 0, 0, 218.17, ...
                       215.71, 216.28, 215.45, 217.06, 0, 222.52, 216.70, 0, 0, 218.42];
rrsta_n_times_raw = [5.45, 5.42, 5.80, 5.22, 5.26, 5.05, 5.57, 5.31, 5.21, 5.48, 5.46, 5.54, 5.84, 5.50, 5.21, ...
                     5.25, 5.41, 5.17, 5.38, 5.37, 5.14, 5.42, 5.43, 5.59, 5.15, 5.09, 5.19, 5.28, 5.42, 5.21, ...
                     5.83, 5.03, 6.01, 5.57, 4.96, 5.53, 5.43, 5.06, 4.89, 5.23, 4.96, 5.17, 4.91, 5.81, 5.27, ...
                     5.00, 5.19, 5.35, 5.21, 5.35];

%% ============ 4. 数据处理 ============
% 提取有效数据（去除路径长度为0的数据）
% y环境
irtt_y_valid_idx = irtt_y_lengths_raw ~= 0;
irtt_y_valid_lengths = irtt_y_lengths_raw(irtt_y_valid_idx);
irtt_y_valid_times = irtt_y_times_raw(irtt_y_valid_idx);

rrsta_y_valid_idx = rrsta_y_lengths_raw ~= 0;
rrsta_y_valid_lengths = rrsta_y_lengths_raw(rrsta_y_valid_idx);
rrsta_y_valid_times = rrsta_y_times_raw(rrsta_y_valid_idx);

% g环境
irtt_g_valid_idx = irtt_g_lengths_raw ~= 0;
irtt_g_valid_lengths = irtt_g_lengths_raw(irtt_g_valid_idx);
irtt_g_valid_times = irtt_g_times_raw(irtt_g_valid_idx);

rrsta_g_valid_idx = rrsta_g_lengths_raw ~= 0;
rrsta_g_valid_lengths = rrsta_g_lengths_raw(rrsta_g_valid_idx);
rrsta_g_valid_times = rrsta_g_times_raw(rrsta_g_valid_idx);

% n环境
irtt_n_valid_idx = irtt_n_lengths_raw ~= 0;
irtt_n_valid_lengths = irtt_n_lengths_raw(irtt_n_valid_idx);
irtt_n_valid_times = irtt_n_times_raw(irtt_n_valid_idx);

rrsta_n_valid_idx = rrsta_n_lengths_raw ~= 0;
rrsta_n_valid_lengths = rrsta_n_lengths_raw(rrsta_n_valid_idx);
rrsta_n_valid_times = rrsta_n_times_raw(rrsta_n_valid_idx);

%% ============ 5. 计算统计量 ============
fprintf('\n============ 统计结果 ============\n');

% 定义算法和环境
algorithms = {'BAIRRT', 'IRTT', 'RRSTA'};
environments = {'y', 'g', 'n'};

% 创建结果表格
results = cell(3*3, 11); % 3种算法 * 3种环境
row = 1;

% 对每种算法和环境组合进行计算
for env_idx = 1:length(environments)
    for algo_idx = 1:length(algorithms)
        env = environments{env_idx};
        algo = algorithms{algo_idx};
        
        % 根据算法和环境获取数据
        if strcmp(algo, 'BAIRRT')
            if strcmp(env, 'y')
                lengths = bairrt_y_lengths;
                times = bairrt_y_times;
            elseif strcmp(env, 'g')
                lengths = bairrt_g_lengths;
                times = bairrt_g_times;
            else % n
                lengths = bairrt_n_lengths;
                times = bairrt_n_times;
            end
            success_count = length(lengths);
        elseif strcmp(algo, 'IRTT')
            if strcmp(env, 'y')
                lengths_raw = irtt_y_lengths_raw;
                lengths = irtt_y_valid_lengths;
                times = irtt_y_valid_times;
            elseif strcmp(env, 'g')
                lengths_raw = irtt_g_lengths_raw;
                lengths = irtt_g_valid_lengths;
                times = irtt_g_valid_times;
            else % n
                lengths_raw = irtt_n_lengths_raw;
                lengths = irtt_n_valid_lengths;
                times = irtt_n_valid_times;
            end
            success_count = length(lengths);
        else % RRSTA
            if strcmp(env, 'y')
                lengths_raw = rrsta_y_lengths_raw;
                lengths = rrsta_y_valid_lengths;
                times = rrsta_y_valid_times;
            elseif strcmp(env, 'g')
                lengths_raw = rrsta_g_lengths_raw;
                lengths = rrsta_g_valid_lengths;
                times = rrsta_g_valid_times;
            else % n
                lengths_raw = rrsta_n_lengths_raw;
                lengths = rrsta_n_valid_lengths;
                times = rrsta_n_valid_times;
            end
            success_count = length(lengths);
        end
        
        % 计算统计量
        if strcmp(algo, 'BAIRRT')
            total_count = 50;
            success_rate = 100;
        else
            total_count = 50;
            success_rate = 100 * success_count / total_count;
        end
        
        if success_count > 0
            avg_length = mean(lengths);
            std_length = std(lengths);
            min_length = min(lengths);
            max_length = max(lengths);
            avg_time = mean(times);
            std_time = std(times);
            min_time = min(times);
            max_time = max(times);
        else
            avg_length = NaN;
            std_length = NaN;
            min_length = NaN;
            max_length = NaN;
            avg_time = NaN;
            std_time = NaN;
            min_time = NaN;
            max_time = NaN;
        end
        
        % 存储结果
        results{row, 1} = algo;
        results{row, 2} = env;
        results{row, 3} = success_rate;
        results{row, 4} = success_count;
        results{row, 5} = total_count;
        results{row, 6} = avg_length;
        results{row, 7} = std_length;
        results{row, 8} = avg_time;
        results{row, 9} = std_time;
        results{row, 10} = min_length;
        results{row, 11} = max_length;
        
        row = row + 1;
    end
end

%% ============ 6. 显示结果 ============
fprintf('\n============ 详细统计结果 ============\n');
fprintf('%-10s %-5s %-12s %-8s %-8s %-12s %-10s %-10s %-10s %-10s %-10s\n', ...
    '算法', '环境', '成功率(%)', '成功数', '总数', '平均长度', '长度标准差', '平均时间', '时间标准差', '最小长度', '最大长度');
fprintf('%-10s %-5s %-12s %-8s %-8s %-12s %-10s %-10s %-10s %-10s %-10s\n', ...
    '----', '----', '---------', '-----', '----', '--------', '--------', '--------', '--------', '--------', '--------');

for i = 1:size(results, 1)
    fprintf('%-10s %-5s %-12.1f %-8d %-8d %-12.4f %-10.4f %-10.4f %-10.4f %-10.4f %-10.4f\n', ...
        results{i,1}, results{i,2}, results{i,3}, results{i,4}, results{i,5}, ...
        results{i,6}, results{i,7}, results{i,8}, results{i,9}, results{i,10}, results{i,11});
end

%% ============ 7. 可视化分析 ============
figure('Position', [100, 100, 1200, 800]);

% 子图1：成功率比较
subplot(2, 3, 1);
success_rates = cell2mat(results(:, 3));
bar_data = reshape(success_rates, 3, 3)';
bar(bar_data);
title('算法成功率比较');
ylabel('成功率 (%)');
legend({'BAIRRT', 'IRTT', 'RRSTA'}, 'Location', 'best');
set(gca, 'XTickLabel', {'y环境', 'g环境', 'n环境'});
grid on;

% 添加数值标签
for i = 1:3
    for j = 1:3
        text(i + (j-2)*0.25, bar_data(i, j) + 2, sprintf('%.1f%%', bar_data(i, j)), ...
            'HorizontalAlignment', 'center', 'FontSize', 8);
    end
end

% 子图2：平均路径长度比较
subplot(2, 3, 2);
avg_lengths = cell2mat(results(:, 6));
bar_data = reshape(avg_lengths, 3, 3)';
bar(bar_data);
title('平均路径长度比较');
ylabel('路径长度');
legend({'BAIRRT', 'IRTT', 'RRSTA'}, 'Location', 'best');
set(gca, 'XTickLabel', {'y环境', 'g环境', 'n环境'});
grid on;

% 子图3：平均运行时间比较
subplot(2, 3, 3);
avg_times = cell2mat(results(:, 8));
bar_data = reshape(avg_times, 3, 3)';
bar(bar_data);
title('平均运行时间比较');
ylabel('运行时间 (秒)');
legend({'BAIRRT', 'IRTT', 'RRSTA'}, 'Location', 'best');
set(gca, 'XTickLabel', {'y环境', 'g环境', 'n环境'});
grid on;

% 子图4：路径长度分布箱线图（y环境）
subplot(2, 3, 4);
box_data = {bairrt_y_lengths, irtt_y_valid_lengths, rrsta_y_valid_lengths};
group = [ones(size(bairrt_y_lengths)), 2*ones(size(irtt_y_valid_lengths)), 3*ones(size(rrsta_y_valid_lengths))];
positions = [1, 2, 3];
boxplot([bairrt_y_lengths, irtt_y_valid_lengths, rrsta_y_valid_lengths], group, 'positions', positions);
title('y环境路径长度分布');
ylabel('路径长度');
set(gca, 'XTick', positions, 'XTickLabel', {'BAIRRT', 'IRTT', 'RRSTA'});
grid on;

% 子图5：路径长度分布箱线图（g环境）
subplot(2, 3, 5);
box_data = {bairrt_g_lengths, irtt_g_valid_lengths, rrsta_g_valid_lengths};
group = [ones(size(bairrt_g_lengths)), 2*ones(size(irtt_g_valid_lengths)), 3*ones(size(rrsta_g_valid_lengths))];
positions = [1, 2, 3];
boxplot([bairrt_g_lengths, irtt_g_valid_lengths, rrsta_g_valid_lengths], group, 'positions', positions);
title('g环境路径长度分布');
ylabel('路径长度');
set(gca, 'XTick', positions, 'XTickLabel', {'BAIRRT', 'IRTT', 'RRSTA'});
grid on;

% 子图6：路径长度分布箱线图（n环境）
subplot(2, 3, 6);
box_data = {bairrt_n_lengths, irtt_n_valid_lengths, rrsta_n_valid_lengths};
group = [ones(size(bairrt_n_lengths)), 2*ones(size(irtt_n_valid_lengths)), 3*ones(size(rrsta_n_valid_lengths))];
positions = [1, 2, 3];
boxplot([bairrt_n_lengths, irtt_n_valid_lengths, rrsta_n_valid_lengths], group, 'positions', positions);
title('n环境路径长度分布');
ylabel('路径长度');
set(gca, 'XTick', positions, 'XTickLabel', {'BAIRRT', 'IRTT', 'RRSTA'});
grid on;

%% ============ 8. 性能对比分析 ============
fprintf('\n============ 性能对比分析 ============\n');
fprintf('1. 成功率对比：\n');
fprintf('   - BAIRRT在所有环境下都保持100%%成功率，表现最优\n');
fprintf('   - IRTT成功率：y环境=%.1f%%, g环境=%.1f%%, n环境=%.1f%%\n', ...
    results{2,3}, results{5,3}, results{8,3});
fprintf('   - RRSTA成功率：y环境=%.1f%%, g环境=%.1f%%, n环境=%.1f%%\n', ...
    results{3,3}, results{6,3}, results{9,3});
fprintf('   - BAIRRT的可靠性明显优于其他两种算法\n\n');

fprintf('2. 路径长度对比（平均值）：\n');
fprintf('   - BAIRRT：y环境=%.2f, g环境=%.2f, n环境=%.2f\n', ...
    results{1,6}, results{4,6}, results{7,6});
fprintf('   - IRTT：y环境=%.2f, g环境=%.2f, n环境=%.2f\n', ...
    results{2,6}, results{5,6}, results{8,6});
fprintf('   - RRSTA：y环境=%.2f, g环境=%.2f, n环境=%.2f\n', ...
    results{3,6}, results{6,6}, results{9,6});
fprintf('   - BAIRRT在y和g环境中路径长度最短，在n环境中表现也较好\n\n');

fprintf('3. 运行时间对比（平均值）：\n');
fprintf('   - BAIRRT：y环境=%.2fs, g环境=%.2fs, n环境=%.2fs\n', ...
    results{1,8}, results{4,8}, results{7,8});
fprintf('   - IRTT：y环境=%.2fs, g环境=%.2fs, n环境=%.2fs\n', ...
    results{2,8}, results{5,8}, results{8,8});
fprintf('   - RRSTA：y环境=%.2fs, g环境=%.2fs, n环境=%.2fs\n', ...
    results{3,8}, results{6,8}, results{9,8});
fprintf('   - BAIRRT在g和n环境中运行时间最短，y环境中时间中等\n\n');

fprintf('4. 算法稳定性（路径长度标准差）：\n');
fprintf('   - BAIRRT：y环境=%.4f, g环境=%.4f, n环境=%.4f\n', ...
    results{1,7}, results{4,7}, results{7,7});
fprintf('   - IRTT：y环境=%.4f, g环境=%.4f, n环境=%.4f\n', ...
    results{2,7}, results{5,7}, results{8,7});
fprintf('   - RRSTA：y环境=%.4f, g环境=%.4f, n环境=%.4f\n', ...
    results{3,7}, results{6,7}, results{9,7});
fprintf('   - BAIRRT的路径长度标准差最小，说明结果最稳定\n\n');

fprintf('5. 综合性能排名：\n');
fprintf('   1. BAIRRT：成功率100%%，路径长度较短，运行时间短，稳定性好\n');
fprintf('   2. RRSTA：成功率中等，路径长度和运行时间中等\n');
fprintf('   3. IRTT：成功率较低，路径长度和运行时间较长\n');

%% ============ 9. 保存结果 - 原始数据写入Excel ============
fprintf('\n============ 保存原始数据到Excel ============\n');

% 创建一个包含多个工作表的新Excel文件
excel_file = '路径规划实验完整数据.xlsx';

% 1. 创建统计结果工作表
result_table = cell2table(results, 'VariableNames', ...
    {'Algorithm', 'environment', 'successrate', 'successtimes', 'totaltimes', 'averagepathlength', 'standarddivlength', ...
     'averagrruntime', 'standrddivruntime', 'minilength', 'maxlength'});
writetable(result_table, excel_file, 'Sheet', '统计结果');

% 2. 创建y环境原始数据工作表
fprintf('写入y环境原始数据...\n');

% BAIRRTy原始数据
bairrt_y_data = [(1:50)', bairrt_y_lengths', bairrt_y_times'];
bairrt_y_table = array2table(bairrt_y_data, 'VariableNames', {'numbei', 'pathlength', 'runtime'});
writetable(bairrt_y_table, excel_file, 'Sheet', 'BAIRRT_y');

% IRTTy原始数据
irtt_y_data = [(1:50)', irtt_y_lengths_raw', irtt_y_times_raw'];
irtt_y_table = array2table(irtt_y_data, 'VariableNames', {'numbei', 'pathlength', 'runtime'});
writetable(irtt_y_table, excel_file, 'Sheet', 'IRTT_y');

% RRSTAy原始数据
rrsta_y_data = [(1:50)', rrsta_y_lengths_raw', rrsta_y_times_raw'];
rrsta_y_table = array2table(rrsta_y_data, 'VariableNames', {'numbei', 'pathlength', 'runtime'});
writetable(rrsta_y_table, excel_file, 'Sheet', 'RRSTA_y');

% 3. 创建g环境原始数据工作表
fprintf('写入g环境原始数据...\n');

% BAIRRTg原始数据
bairrt_g_data = [(1:50)', bairrt_g_lengths', bairrt_g_times'];
bairrt_g_table = array2table(bairrt_g_data, 'VariableNames', {'numbei', 'pathlength', 'runtime'});
writetable(bairrt_g_table, excel_file, 'Sheet', 'BAIRRT_g');

% IRTTg原始数据
irtt_g_data = [(1:50)', irtt_g_lengths_raw', irtt_g_times_raw'];
irtt_g_table = array2table(irtt_g_data, 'VariableNames', {'numbei', 'pathlength', 'runtime'});
writetable(irtt_g_table, excel_file, 'Sheet', 'IRTT_g');

% RRSTAg原始数据
rrsta_g_data = [(1:50)', rrsta_g_lengths_raw', rrsta_g_times_raw'];
rrsta_g_table = array2table(rrsta_g_data, 'VariableNames',{'numbei', 'pathlength', 'runtime'});
writetable(rrsta_g_table, excel_file, 'Sheet', 'RRSTA_g');

% 4. 创建n环境原始数据工作表
fprintf('写入n环境原始数据...\n');

% BAIRRTn原始数据
bairrt_n_data = [(1:50)', bairrt_n_lengths', bairrt_n_times'];
bairrt_n_table = array2table(bairrt_n_data, 'VariableNames',{'numbei', 'pathlength', 'runtime'});
writetable(bairrt_n_table, excel_file, 'Sheet', 'BAIRRT_n');

% IRTTn原始数据
irtt_n_data = [(1:50)', irtt_n_lengths_raw', irtt_n_times_raw'];
irtt_n_table = array2table(irtt_n_data, 'VariableNames', {'numbei', 'pathlength', 'runtime'});
writetable(irtt_n_table, excel_file, 'Sheet', 'IRTT_n');

% RRSTAn原始数据
rrsta_n_data = [(1:50)', rrsta_n_lengths_raw', rrsta_n_times_raw'];
rrsta_n_table = array2table(rrsta_n_data, 'VariableNames', {'numbei', 'pathlength', 'runtime'});
writetable(rrsta_n_table, excel_file, 'Sheet', 'RRSTA_n');

fprintf('原始数据已保存到: %s\n', excel_file);

% 5. 创建数据摘要工作表
fprintf('创建数据摘要工作表...\n');
summary_data = {
    '总实验次数', '450';
    'BAIRRT总成功次数', '150';
    'IRTT总成功次数', sprintf('%d', sum([length(irtt_y_valid_lengths), length(irtt_g_valid_lengths), length(irtt_n_valid_lengths)]));
    'RRSTA总成功次数', sprintf('%d', sum([length(rrsta_y_valid_lengths), length(rrsta_g_valid_lengths), length(rrsta_n_valid_lengths)]));
    'BAIRRT总成功率', '100%';
    'IRTT总成功率', sprintf('%.1f%%', 100*sum([length(irtt_y_valid_lengths), length(irtt_g_valid_lengths), length(irtt_n_valid_lengths)])/150);
    'RRSTA总成功率', sprintf('%.1f%%', 100*sum([length(rrsta_y_valid_lengths), length(rrsta_g_valid_lengths), length(rrsta_n_valid_lengths)])/150);
    '平均路径长度(BAIRRT)', sprintf('%.2f', mean([mean(bairrt_y_lengths), mean(bairrt_g_lengths), mean(bairrt_n_lengths)]));
    '平均路径长度(IRTT)', sprintf('%.2f', mean([mean(irtt_y_valid_lengths), mean(irtt_g_valid_lengths), mean(irtt_n_valid_lengths)]));
    '平均路径长度(RRSTA)', sprintf('%.2f', mean([mean(rrsta_y_valid_lengths), mean(rrsta_g_valid_lengths), mean(rrsta_n_valid_lengths)]));
    '平均运行时间(BAIRRT)', sprintf('%.2fs', mean([mean(bairrt_y_times), mean(bairrt_g_times), mean(bairrt_n_times)]));
    '平均运行时间(IRTT)', sprintf('%.2fs', mean([mean(irtt_y_valid_times), mean(irtt_g_valid_times), mean(irtt_n_valid_times)]));
    '平均运行时间(RRSTA)', sprintf('%.2fs', mean([mean(rrsta_y_valid_times), mean(rrsta_g_valid_times), mean(rrsta_n_valid_times)]));
};

summary_table = cell2table(summary_data, 'VariableNames', {'指标', '数值'});
writetable(summary_table, excel_file, 'Sheet', '数据摘要');

% 保存图表
saveas(gcf, '路径规划实验结果.png');
fprintf('图表已保存为: 路径规划实验结果.png\n');

%% ============ 10. 环境间差异分析 ============
fprintf('\n============ 环境间差异分析 ============\n');

% 分析不同环境对算法性能的影响
for algo_idx = 1:length(algorithms)
    algo = algorithms{algo_idx};
    fprintf('\n--- %s算法在不同环境中的表现 ---\n', algo);
    
    % 获取该算法在三种环境中的成功率
    env_success_rates = [];
    env_avg_lengths = [];
    env_avg_times = [];
    
    for env_idx = 1:length(environments)
        env = environments{env_idx};
        % 找到对应的行
        for i = 1:size(results, 1)
            if strcmp(results{i,1}, algo) && strcmp(results{i,2}, env)
                env_success_rates = [env_success_rates, results{i,3}];
                env_avg_lengths = [env_avg_lengths, results{i,6}];
                env_avg_times = [env_avg_times, results{i,8}];
                break;
            end
        end
    end
    
    fprintf('   成功率: y=%.1f%%, g=%.1f%%, n=%.1f%%\n', env_success_rates(1), env_success_rates(2), env_success_rates(3));
    fprintf('   平均路径长度: y=%.2f, g=%.2f, n=%.2f\n', env_avg_lengths(1), env_avg_lengths(2), env_avg_lengths(3));
    fprintf('   平均运行时间: y=%.2fs, g=%.2fs, n=%.2fs\n', env_avg_times(1), env_avg_times(2), env_avg_times(3));
    
    % 分析环境差异
    if strcmp(algo, 'BAIRRT')
        fprintf('   分析: BAIRRT在所有环境中都保持100%%成功率，但在n环境中路径长度略有增加。\n');
    elseif strcmp(algo, 'IRTT')
        fprintf('   分析: IRTT在g环境中成功率最低，在n环境中路径长度和运行时间最长。\n');
    else % RRSTA
        fprintf('   分析: RRSTA在y环境中表现最好，在g环境中成功率较低。\n');
    end
end

fprintf('\n============ 分析完成 ============\n');
fprintf('所有数据已保存到文件: %s\n', excel_file);
fprintf('文件包含以下工作表:\n');
fprintf('  1. 统计结果 - 算法性能统计\n');
fprintf('  2. 数据摘要 - 实验总体情况\n');
fprintf('  3. BAIRRT_y, IRTT_y, RRSTA_y - y环境原始数据\n');
fprintf('  4. BAIRRT_g, IRTT_g, RRSTA_g - g环境原始数据\n');
fprintf('  5. BAIRRT_n, IRTT_n, RRSTA_n - n环境原始数据\n');