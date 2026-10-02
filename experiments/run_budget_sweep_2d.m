function run_budget_sweep_2d()
%RUN_BUDGET_SWEEP_2D 二维预算敏感性：同一套算法在更低预算下的表现（50 次配对重复）。
%   目的：回答“降低采样次数是否会改变结论”。预算越小，热启动带来的优势应当越明显
%   （AB-IRRT* 的初始解不依赖采样预算），但快速可行解类基线也可能在低预算下更早给出解，
%   因此用实验而不是猜测来判断。
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir); addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));

budgets = [500, 1000];
for b = budgets
    out = fullfile(entryDir, sprintf('results_2d_b%d.csv', b));
    at = fullfile(entryDir, sprintf('anytime_2d_b%d.mat', b));
    if exist(out, 'file')
        log_progress('budget sweep 2D: %d already done, skip', b);
        continue;
    end
    log_progress('budget sweep 2D: budget=%d start', b);
    run_2d_comparison(50, struct('maxIter', b, 'outputFile', out, ...
        'anytimeFile', at, 'quiet', false));
    log_progress('budget sweep 2D: budget=%d done', b);
end
log_progress('=== run_budget_sweep_2d DONE ===');
end
