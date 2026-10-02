function hybrid_run_all(runs2d, runs3d)
%HYBRID_RUN_ALL 依次运行二维与三维的文献混合基线组实验。
if nargin < 1 || isempty(runs2d), runs2d = 50; end
if nargin < 2 || isempty(runs3d), runs3d = 50; end
hybrid_run2d(runs2d);
hybrid_run3d(runs3d);
fprintf('hybrid_run_all done\n');
end
