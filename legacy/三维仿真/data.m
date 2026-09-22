% 清除环境
clear; clc;

% BAIRRT 数据
bairrt_length = [271.16, 271.58, 271.22, 270.57, 271.25, 273.66, 271.65, 272.32, 273.46, 270.75, ...
                 271.34, 272.13, 271.38, 272.53, 270.48, 272.19, 270.39, 270.36, 271.27, 271.27, ...
                 270.39, 272.17, 271.71, 272.61, 272.77, 271.46, 271.99, 272.07, 271.24, 271.50, ...
                 270.80, 270.48, 271.68, 271.04, 270.79, 272.48, 271.04, 271.62, 270.39, 271.74, ...
                 271.56, 271.04, 272.19, 271.77, 271.67, 271.59, 275.67, 271.93, 271.75, 271.17];

bairrt_time = [17.60, 17.34, 18.48, 17.31, 17.42, 16.25, 16.71, 17.74, 16.94, 16.94, ...
               17.22, 16.74, 17.36, 16.10, 17.95, 17.84, 17.67, 18.15, 17.27, 17.83, ...
               18.11, 17.39, 17.49, 17.17, 17.48, 17.85, 17.25, 18.13, 17.50, 17.53, ...
               16.96, 17.86, 17.72, 17.86, 17.49, 16.51, 18.50, 17.70, 17.78, 17.44, ...
               17.63, 18.27, 17.16, 17.56, 17.68, 17.48, 17.21, 17.78, 18.12, 17.90];

% IRRT* 数据
irrt_length = [275.53, 279.80, 277.69, Inf, 275.57, 275.45, 286.30, 280.82, Inf, 277.93, ...
               294.72, 281.15, 284.89, 278.25, 278.90, 281.05, Inf, 281.10, 280.35, Inf, ...
               280.20, 280.15, 279.24, 277.26, 281.52, 276.78, 283.83, 287.65, 286.04, 283.95, ...
               278.89, 283.82, 280.17, 275.28, 279.46, 277.39, 284.44, 277.67, 287.12, 275.92, ...
               279.44, 274.11, 275.34, 275.37, 276.37, Inf, 280.48, 280.95, Inf, 277.26];

irrt_time = [17.25, 18.18, 17.11, 13.62, 17.06, 17.20, 14.35, 15.24, 13.99, 17.17, ...
             14.33, 18.28, 17.23, 16.72, 16.38, 16.51, 14.44, 15.16, 16.04, 13.71, ...
             17.27, 16.30, 16.77, 17.73, 16.74, 16.58, 17.46, 16.92, 15.69, 15.55, ...
             16.99, 16.68, 18.09, 16.69, 16.32, 18.41, 14.25, 17.55, 14.81, 17.96, ...
             16.76, 18.03, 17.50, 17.00, 17.72, 13.92, 15.61, 18.10, 14.06, 17.14];

% RTS 数据
rts_length = [305.96, Inf, 289.01, Inf, 290.87, Inf, 285.20, 288.22, 292.32, 303.76, ...
              Inf, Inf, Inf, 301.33, Inf, Inf, 294.14, 293.45, 293.78, 293.86, ...
              293.12, 290.52, 288.78, Inf, 291.74, 304.87, 294.32, 293.77, Inf, 289.71, ...
              304.17, Inf, 297.23, Inf, 295.84, Inf, 297.12, 307.70, 284.48, 288.71, ...
              297.31, 289.64, 282.83, 290.23, 290.45, Inf, 293.78, 288.07, 284.77, 302.51];

rts_time = [14.89, 13.96, 14.60, 13.91, 14.62, 14.55, 13.37, 14.03, 14.81, 14.85, ...
            14.21, 14.96, 14.91, 15.52, 14.67, 14.78, 14.10, 14.89, 14.24, 14.14, ...
            14.31, 14.06, 14.60, 14.93, 14.12, 15.30, 14.28, 14.08, 14.18, 14.36, ...
            15.82, 15.06, 14.56, 14.21, 14.50, 15.02, 14.37, 15.34, 13.90, 14.52, ...
            14.78, 14.76, 13.90, 13.95, 14.64, 14.85, 14.74, 14.24, 14.79, 14.82];

% 计算统计信息
algorithms = {'BAIRRT', 'IRRT*', 'RTS'};

% 成功率计算
bairrt_success_rate = 100 * sum(~isinf(bairrt_length)) / length(bairrt_length);
irrt_success_rate = 100 * sum(~isinf(irrt_length)) / length(irrt_length);
rts_success_rate = 100 * sum(~isinf(rts_length)) / length(rts_length);

% 有效数据索引（去除Inf）
bairrt_valid = ~isinf(bairrt_length);
irrt_valid = ~isinf(irrt_length);
rts_valid = ~isinf(rts_length);

% 路径长度统计（仅有效数据）
bairrt_length_stats = [mean(bairrt_length(bairrt_valid)), std(bairrt_length(bairrt_valid)), ...
                      min(bairrt_length(bairrt_valid)), max(bairrt_length(bairrt_valid))];
irrt_length_stats = [mean(irrt_length(irrt_valid)), std(irrt_length(irrt_valid)), ...
                    min(irrt_length(irrt_valid)), max(irrt_length(irrt_valid))];
rts_length_stats = [mean(rts_length(rts_valid)), std(rts_length(rts_valid)), ...
                   min(rts_length(rts_valid)), max(rts_length(rts_valid))];

% 时间统计（仅有效数据）
bairrt_time_stats = [mean(bairrt_time(bairrt_valid)), std(bairrt_time(bairrt_valid)), ...
                    min(bairrt_time(bairrt_valid)), max(bairrt_time(bairrt_valid))];
irrt_time_stats = [mean(irrt_time(irrt_valid)), std(irrt_time(irrt_valid)), ...
                  min(irrt_time(irrt_valid)), max(irrt_time(irrt_valid))];
rts_time_stats = [mean(rts_time(rts_valid)), std(rts_time(rts_valid)), ...
                 min(rts_time(rts_valid)), max(rts_time(rts_valid))];

% 创建结果表格
results_table = table();

% 算法名称
results_table.Algorithm = algorithms';

% 成功率
results_table.SuccessRate = [bairrt_success_rate; irrt_success_rate; rts_success_rate];

% 路径长度统计
results_table.MeanPathLength = [bairrt_length_stats(1); irrt_length_stats(1); rts_length_stats(1)];
results_table.StdPathLength = [bairrt_length_stats(2); irrt_length_stats(2); rts_length_stats(2)];
results_table.MinPathLength = [bairrt_length_stats(3); irrt_length_stats(3); rts_length_stats(3)];
results_table.MaxPathLength = [bairrt_length_stats(4); irrt_length_stats(4); rts_length_stats(4)];

% 时间统计
results_table.MeanTime = [bairrt_time_stats(1); irrt_time_stats(1); rts_time_stats(1)];
results_table.StdTime = [bairrt_time_stats(2); irrt_time_stats(2); rts_time_stats(2)];
results_table.MinTime = [bairrt_time_stats(3); irrt_time_stats(3); rts_time_stats(3)];
results_table.MaxTime = [bairrt_time_stats(4); irrt_time_stats(4); rts_time_stats(4)];

% 成功次数
results_table.SuccessCount = [sum(bairrt_valid); sum(irrt_valid); sum(rts_valid)];
results_table.TotalRuns = [length(bairrt_length); length(irrt_length); length(rts_length)];

% 显示结果
disp('算法性能对比统计:');
disp(results_table);

% 保存到Excel文件
filename = 'Path_Planning_Results.xlsx';
writetable(results_table, filename, 'Sheet', 'Statistical_Summary');

% 创建详细数据表格
detailed_data = table();

% 运行次数
runs = (1:50)';
detailed_data.Run = runs;

% BAIRRT数据
detailed_data.BAIRRT_Length = bairrt_length';
detailed_data.BAIRRT_Time = bairrt_time';

% IRRT*数据
detailed_data.IRRT_Length = irrt_length';
detailed_data.IRRT_Time = irrt_time';

% RTS数据
detailed_data.RTS_Length = rts_length';
detailed_data.RTS_Time = rts_time';

% 保存详细数据到Excel的另一个sheet
writetable(detailed_data, filename, 'Sheet', 'Detailed_Data');

% 创建性能对比图表
figure('Position', [100, 100, 1200, 800]);

% 子图1: 路径长度对比
subplot(2,3,1);
box_data = [bairrt_length(bairrt_valid)', irrt_length(irrt_valid)', rts_length(rts_valid)'];
boxplot(box_data, 'Labels', algorithms);
title('路径长度分布');
ylabel('路径长度');
grid on;

% 子图2: 运行时间对比
subplot(2,3,2);
box_time = [bairrt_time(bairrt_valid)', irrt_time(irrt_valid)', rts_time(rts_valid)'];
boxplot(box_time, 'Labels', algorithms);
title('运行时间分布');
ylabel('时间 (秒)');
grid on;

% 子图3: 成功率对比
subplot(2,3,3);
success_rates = [bairrt_success_rate, irrt_success_rate, rts_success_rate];
bar(success_rates);
set(gca, 'XTickLabel', algorithms);
title('成功率对比');
ylabel('成功率 (%)');
grid on;

% 子图4: 路径长度随时间变化
subplot(2,3,4);
plot(runs, bairrt_length, 'b-', 'LineWidth', 1.5); hold on;
plot(runs, irrt_length, 'r-', 'LineWidth', 1.5);
plot(runs, rts_length, 'g-', 'LineWidth', 1.5);
legend('BAIRRT', 'IRRT*', 'RTS', 'Location', 'best');
title('路径长度变化趋势');
xlabel('运行次数');
ylabel('路径长度');
grid on;

% 子图5: 运行时间随时间变化
subplot(2,3,5);
plot(runs, bairrt_time, 'b-', 'LineWidth', 1.5); hold on;
plot(runs, irrt_time, 'r-', 'LineWidth', 1.5);
plot(runs, rts_time, 'g-', 'LineWidth', 1.5);
legend('BAIRRT', 'IRRT*', 'RTS', 'Location', 'best');
title('运行时间变化趋势');
xlabel('运行次数');
ylabel('时间 (秒)');
grid on;

% 子图6: 性能综合评分（成功率/平均时间）
subplot(2,3,6);
performance_score = [bairrt_success_rate/bairrt_time_stats(1), ...
                    irrt_success_rate/irrt_time_stats(1), ...
                    rts_success_rate/rts_time_stats(1)];
bar(performance_score);
set(gca, 'XTickLabel', algorithms);
title('性能综合评分 (成功率/平均时间)');
ylabel('评分');
grid on;

% 保存图表
saveas(gcf, 'Algorithm_Comparison.png');
disp(' ');
disp(['结果已保存到文件: ' filename]);
disp('图表已保存为: Algorithm_Comparison.png');

% 输出关键结论
fprintf('\n=== 关键结论 ===\n');
fprintf('1. BAIRRT算法表现最佳:\n');
fprintf('   - 成功率: %.1f%% (50/50)\n', bairrt_success_rate);
fprintf('   - 平均路径长度: %.2f\n', bairrt_length_stats(1));
fprintf('   - 平均运行时间: %.2f秒\n', bairrt_time_stats(1));

fprintf('2. IRRT*算法:\n');
fprintf('   - 成功率: %.1f%% (%d/50)\n', irrt_success_rate, sum(irrt_valid));
fprintf('   - 平均路径长度: %.2f\n', irrt_length_stats(1));

fprintf('3. RTS算法:\n');
fprintf('   - 成功率: %.1f%% (%d/50)\n', rts_success_rate, sum(rts_valid));
fprintf('   - 运行时间最短: %.2f秒\n', rts_time_stats(1));