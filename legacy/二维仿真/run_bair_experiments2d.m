function results = run_bair_experiments2d(runs, opts)
%RUN_BAIR_EXPERIMENTS2D Reproducible BAIR benchmark on y/g/n environments.
if nargin < 1 || isempty(runs), runs = 50; end
if nargin < 2, opts = struct; end
if ~isfield(opts,'outputFile'), opts.outputFile = 'bair_results_2d.csv'; end
modes = {'y','g','n'};
results = struct('mode',{},'run',{},'success',{},'length',{},'time',{},'apfLength',{},'nodes',{});
z = 0;
for im = 1:numel(modes)
    for r = 1:runs
        rng(20231202 + 100*im + r, 'twister');
        [~,~,d] = bair_core2d(modes{im}, opts);
        z = z + 1;
        results(z) = struct('mode',modes{im},'run',r,'success',d.success, ...
            'length',d.length,'time',d.time,'apfLength',d.apf.length,'nodes',d.rrt.nodes);
    end
end
if ~isempty(opts.outputFile)
    writetable(struct2table(results), opts.outputFile);
end
end
