function warm_smoke()
%WARM_SMOKE 快速验证热启动等价基线可跑通（2 维，3 场景 × 2 次，不覆盖正式结果）。
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir);
addpath(fullfile(rootDir, 'algorithms_2d'));
addpath(fullfile(rootDir, 'algorithms_3d'));

R = run_warmstart_comparison(2, 2);
fprintf('rows = %d\n', height(R));

algs = unique(R.algorithm, 'stable');
scenes = unique(R.scenario, 'stable');
for s = 1:numel(scenes)
    fprintf('--- %s ---\n', scenes{s});
    for i = 1:numel(algs)
        m = strcmp(R.algorithm, algs{i}) & strcmp(R.scenario, scenes{s});
        if ~any(m), continue; end
        fprintf('  %-20s ok=%.2f  len=%8.2f  t=%8.1f ms  firstT=%7.1f ms\n', algs{i}, ...
            mean(R.success(m)), mean(R.length(m), 'omitnan'), ...
            1000 * mean(R.time(m), 'omitnan'), 1000 * mean(R.firstSolutionTime(m), 'omitnan'));
    end
end
end
