function [results2d, results3d] = run_all_comparisons(runs2d, runs3d, opts)
%RUN_ALL_COMPARISONS One-click entry for the complete comparison suite.
if nargin<1||isempty(runs2d),runs2d=10;end
if nargin<2||isempty(runs3d),runs3d=1;end
if nargin<3,opts=struct;end
entryDir=fileparts(mfilename('fullpath')); addpath(entryDir);
opts2d=opts; opts3d=opts;
if ~isfield(opts2d,'outputFile') || isempty(opts2d.outputFile)
    opts2d.outputFile=fullfile(entryDir,'results_2d.csv');
end
if ~isfield(opts3d,'outputFile') || isempty(opts3d.outputFile)
    opts3d.outputFile=fullfile(entryDir,'results_3d.csv');
else
    [p,n,e]=fileparts(opts3d.outputFile);
    opts3d.outputFile=fullfile(p,[n '_3d' e]);
end
results2d=run_2d_comparison(runs2d,opts2d);
results3d=run_3d_comparison(runs3d,opts3d);
end
