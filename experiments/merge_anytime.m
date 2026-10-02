function merge_anytime
%MERGE_ANYTIME 把 rerun_bair 生成的 AB-IRRT* anytime 历史合并回主 anytime 文件。
% 用法：_run_ml.ps1 -Expr merge_anytime
entryDir = fileparts(mfilename('fullpath'));

% 二维：单个主文件，内含三个场景
mergeOne(fullfile(entryDir, 'anytime_2d_v2.mat'), ...
    fullfile(entryDir, 'anytime_bair_2d.mat'));

% 三维：逐场景主文件（anytime_3d_v2_<scene>.mat）
for s = {'general', 'narrow', 'suspended', 'ring', 'overhang'}
    mergeOne(fullfile(entryDir, sprintf('anytime_3d_v2_%s.mat', s{1})), ...
        fullfile(entryDir, 'anytime_bair_3d.mat'));
end
end

function mergeOne(main, add)
[~, name, ext] = fileparts(main);
name = [name, ext];
if ~exist(add, 'file')
    fprintf('SKIP %s（缺少 %s）\n', name, add);
    return;
end
if ~exist(main, 'file')
    fprintf('SKIP %s（主文件不存在）\n', name);
    return;
end
S = load(main, 'anytime');
anytime = S.anytime;
A = load(add, 'anytime');
anNew = A.anytime;
% 主文件只含单一场景时，只取该场景的新记录（三维逐场景文件的情况）
scenes = unique({anytime.scenario});
if numel(scenes) == 1
    anNew = anNew(strcmp({anNew.scenario}, scenes{1}));
end
% 备份（只备份第一次合并前的状态）
bak = [main, '.bak'];
if ~exist(bak, 'file'), copyfile(main, bak); end
% 删除旧的 AB-IRRT* 记录
anytime = anytime(~strcmp({anytime.algorithm}, 'AB-IRRT*'));
% 追加新的
for i = 1:numel(anNew)
    anytime(end + 1) = anNew(i); %#ok<AGROW>
end
save(main, 'anytime', '-v7.3');
fprintf('%s: 合并 %d 条新记录 → 共 %d 条 anytime 记录（AB-IRRT* %d 条）\n', name, ...
    numel(anNew), numel(anytime), sum(strcmp({anytime.algorithm}, 'AB-IRRT*')));
end
