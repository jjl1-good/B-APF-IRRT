function run_ge_3d(runs)
%RUN_GE_3D 只跑"葛超等 2025（改进 Informed-RRT*）"的三维推广版，与主实验同场景、同种子、同预算。
%   三维逐场景单独调用（c=1），使种子与主实验/混合基线组完全一致（20260903+1000·1+r）。
%   输出：ge_2025_3d_<tag>.csv 与 ge_2025_3d_uniform_<tag>.csv（逐场景），
%         正式表由 _merge_ge_3d.py 合并为 ge_2025_3d.csv / ge_2025_3d_uniform.csv。
if nargin < 1 || isempty(runs), runs = 50; end
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir); addpath(fullfile(rootDir, 'common')); addpath(fullfile(rootDir, 'common3d'));

scenes = {'general_v2', 'narrow', 'suspended', 'ring', 'overhang'};
tags = {'general', 'narrow', 'suspended', 'ring', 'overhang'};
variants = {{'Ge-IRRT*', 'ge_2025_3d'}, {'Ge-IRRT*-uniform', 'ge_2025_3d_uniform'}};

for v = 1:numel(variants)
    alg = variants{v}{1}; stem = variants{v}{2};
    for i = 1:numel(scenes)
        o = struct('algs', {{alg}}, 'quiet', true, 'scenes', {{scenes{i}}}, ...
            'outputFile', fullfile(entryDir, sprintf('%s_%s.csv', stem, tags{i})), ...
            'anytimeFile', fullfile(entryDir, sprintf('%s_anytime_%s.mat', stem, tags{i})));
        fprintf('--- %s / %s ---\n', alg, tags{i});
        run_3d_comparison(runs, o);
    end
end
fprintf('ge 3-D comparison done (%d runs)\n', runs);
end
