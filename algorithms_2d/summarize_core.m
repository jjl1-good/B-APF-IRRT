function summary = summarize_core(T, refName, outputFile)
%SUMMARIZE_CORE 通用统计汇总（Bootstrap CI / 效应量 / 配对置换检验）。
%
%   summary = summarize_core(T, refName)
%   summary = summarize_core(T, refName, outputFile)
%
% 输入 T 为 run_2d/3d_comparison 输出的原始明细表（每行一次运行）。
% 对每个 (scenario, algorithm) 组合计算：
%   successes, successRate(+CI), mean/std/median 路径长度(+CI),
%   mean/std 时间(+CI), mean firstSolutionTime(+CI), meanMinClear,
%   meanTurnSum, meanNodes，
% 以及相对参考算法（默认 AB-IRRT*）的 Cliff's delta 效应量与
% 配对置换检验 p 值（同一 run 号配对；长度/时间/首解时间）。
% 结果写入 CSV（UTF-8）并在命令行打印 Markdown 汇总表。

if nargin < 2 || isempty(refName), refName = 'AB-IRRT*'; end
if nargin < 3, outputFile = ''; end
if ischar(T) || (isstring(T) && isscalar(T)), T = readtable(T); end
vars = T.Properties.VariableNames;
scenarios = unique(T.scenario, 'stable');
algorithms = unique(T.algorithm, 'stable');
nSc = numel(scenarios); nAlg = numel(algorithms);

summary = struct('scenario', {}, 'algorithm', {}, 'runs', {}, 'successes', {}, ...
    'successRate', {}, 'successCIlo', {}, 'successCIhi', {}, ...
    'meanLength', {}, 'stdLength', {}, 'medianLength', {}, 'lenCIlo', {}, 'lenCIhi', {}, ...
    'meanTime', {}, 'stdTime', {}, 'timeCIlo', {}, 'timeCIhi', {}, ...
    'meanFirstSol', {}, 'firstCIlo', {}, 'firstCIhi', {}, ...
    'meanMinClear', {}, 'meanTurnSum', {}, 'meanNodes', {}, ...
    'meanPre', {}, 'meanOpt', {}, 'meanEscapes', {}, 'meanFlips', {}, 'fallbackRate', {}, ...
    'deltaLength', {}, 'pLength', {}, 'deltaTime', {}, 'pTime', {}, ...
    'deltaFirst', {}, 'pFirst', {});
z = 0;
for i = 1:nSc
    for j = 1:nAlg
        mask = strcmp(T.scenario, scenarios{i}) & strcmp(T.algorithm, algorithms{j});
        A = T(mask, :);
        success = tget(A, vars, 'success', 0);
        len = tget(A, vars, 'length', NaN); len = len(success == 1 & isfinite(len));
        tm = tget(A, vars, 'time', NaN); tm = tm(success == 1 & isfinite(tm));
        fsol = tget(A, vars, 'firstSolutionTime', NaN); fsol = fsol(success == 1 & isfinite(fsol));
        clear = tget(A, vars, 'minClear', NaN); clear = clear(success == 1 & isfinite(clear));
        turn = tget(A, vars, 'turnSum', NaN); turn = turn(success == 1 & isfinite(turn));
        nodes = tget(A, vars, 'nodes', NaN); nodes = nodes(isfinite(nodes));
        preT = tget(A, vars, 'tPre', NaN); preT = preT(success == 1 & isfinite(preT));
        optT = tget(A, vars, 'tOpt', NaN); optT = optT(success == 1 & isfinite(optT));
        esc = tget(A, vars, 'escapes', NaN); esc = esc(success == 1 & isfinite(esc));
        flp = tget(A, vars, 'sideFlips', NaN); flp = flp(success == 1 & isfinite(flp));
        fbr = tget(A, vars, 'fallback', 0); fbr = fbr(success == 1);

        [sLo, sHi] = bootstrap_ci(success);
        [lLo, lHi] = safe_ci(len);
        [tLo, tHi] = safe_ci(tm);
        [fLo, fHi] = safe_ci(fsol);

        % 相对参考算法的配对比较（同 run 号）
        refMask = strcmp(T.scenario, scenarios{i}) & strcmp(T.algorithm, refName);
        R = T(refMask, :);
        [dL, pL] = paired_compare(tget(A, vars, 'run', NaN), tget(A, vars, 'success', 0), tget(A, vars, 'length', NaN), ...
            tget(R, vars, 'run', NaN), tget(R, vars, 'success', 0), tget(R, vars, 'length', NaN));
        [dT, pT] = paired_compare(tget(A, vars, 'run', NaN), tget(A, vars, 'success', 0), tget(A, vars, 'time', NaN), ...
            tget(R, vars, 'run', NaN), tget(R, vars, 'success', 0), tget(R, vars, 'time', NaN));
        [dF, pF] = paired_compare(tget(A, vars, 'run', NaN), tget(A, vars, 'success', 0), tget(A, vars, 'firstSolutionTime', NaN), ...
            tget(R, vars, 'run', NaN), tget(R, vars, 'success', 0), tget(R, vars, 'firstSolutionTime', NaN));

        z = z + 1;
        summary(z) = struct('scenario', scenarios{i}, 'algorithm', algorithms{j}, ...
            'runs', height(A), 'successes', sum(success == 1), ...
            'successRate', 100 * mean(success), 'successCIlo', 100 * sLo, 'successCIhi', 100 * sHi, ...
            'meanLength', mean_or(len, inf), 'stdLength', std_or(len), 'medianLength', med_or(len), ...
            'lenCIlo', lLo, 'lenCIhi', lHi, ...
            'meanTime', mean_or(tm, inf), 'stdTime', std_or(tm), 'timeCIlo', tLo, 'timeCIhi', tHi, ...
            'meanFirstSol', mean_or(fsol, inf), 'firstCIlo', fLo, 'firstCIhi', fHi, ...
            'meanMinClear', mean_or(clear, NaN), 'meanTurnSum', mean_or(turn, NaN), ...
            'meanNodes', mean_or(nodes, NaN), ...
            'meanPre', mean_or(preT, NaN), 'meanOpt', mean_or(optT, NaN), ...
            'meanEscapes', mean_or(esc, NaN), 'meanFlips', mean_or(flp, NaN), ...
            'fallbackRate', 100 * mean_or(fbr, NaN), ...
            'deltaLength', dL, 'pLength', pL, 'deltaTime', dT, 'pTime', pT, ...
            'deltaFirst', dF, 'pFirst', pF);
    end
    % 场景内打印 Markdown 表（论文/回复信可直接使用）
    fprintf('\n### %s\n\n', scenarios{i});
    fprintf('| Algorithm | Success%% | Length(mean±std) | CI95 | Time(s) | FirstSol(s) | minClear | vs ref Δ/p |\n');
    fprintf('|---|---|---|---|---|---|---|---|\n');
    for j = 1:nAlg
        s = summary(z - nAlg + j);
        fprintf('| %s | %.1f | %.1f ± %.1f | [%.1f, %.1f] | %.2f | %.2f | %.1f | Δ=%.2f, p=%.3f |\n', ...
            s.algorithm, s.successRate, s.meanLength, s.stdLength, s.lenCIlo, s.lenCIhi, ...
            s.meanTime, s.meanFirstSol, s.meanMinClear, s.deltaLength, s.pLength);
    end
end
summary = struct2table(summary);
if ~isempty(outputFile)
    writetable(summary, outputFile, 'Encoding', 'UTF-8');
    fprintf('\nSummary written: %s\n', outputFile);
end
end

function v = tget(T, vars, name, def)
if any(strcmp(vars, name))
    v = double(T.(name));
    v = v(:);
else
    v = repmat(def, height(T), 1);
end
end

function [d, p] = paired_compare(runsA, succA, valsA, runsB, succB, valsB)
% 按 run 号配对：仅使用双方均成功且值有限的配对。
d = NaN; p = NaN;
mapB = containers.Map(runsB(:)', num2cell(1:numel(runsB)));
xa = []; xb = [];
for i = 1:numel(runsA)
    if ~succA(i) || ~isfinite(valsA(i)), continue; end
    k = runsA(i);
    if ~isKey(mapB, k), continue; end
    j = mapB(k);
    if ~succB(j) || ~isfinite(valsB(j)), continue; end
    xa(end + 1, 1) = valsA(i); %#ok<AGROW>
    xb(end + 1, 1) = valsB(j); %#ok<AGROW>
end
if numel(xa) >= 3
    d = cliffs_delta(xa, xb);
    p = permutation_test(xa, xb, struct('paired', true, 'nPerm', 4000));
end
end

function [lo, hi] = safe_ci(x)
if numel(x) >= 3
    [lo, hi] = bootstrap_ci(x);
else
    lo = NaN; hi = NaN;
end
end

function v = mean_or(x, d)
if isempty(x), v = d; else, v = mean(x); end
end

function v = std_or(x)
if numel(x) < 2, v = 0; else, v = std(x); end
end

function v = med_or(x)
if isempty(x), v = NaN; else, v = median(x); end
end
