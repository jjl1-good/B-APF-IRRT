function regen_all_figures()
%REGEN_ALL_FIGURES 重新生成全部论文图件（英文题注、配色统一）。
%   1) make_revision_figures : 箱线图 / 成功率 / 时间 / 首解 / anytime / 二维采样 / 三维场景
%   2) make_algorithm_figures: 九算法路径对比、采样密集度热图（2D/3D）、消融对比图
entryDir = fileparts(mfilename('fullpath'));
rootDir = fileparts(entryDir);
addpath(entryDir); addpath(fullfile(rootDir, 'algorithms_2d')); addpath(fullfile(rootDir, 'algorithms_3d'));

log_progress('=== regen_all_figures: make_revision_figures ===');
make_revision_figures();

log_progress('=== regen_all_figures: make_algorithm_figures ===');
make_algorithm_figures();

log_progress('=== regen_all_figures DONE ===');
end
