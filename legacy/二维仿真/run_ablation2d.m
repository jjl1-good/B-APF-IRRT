function results = run_ablation2d(runs, opts)
%RUN_ABLATION2D Compare APF, RRT*, Informed-RRT*, BAIR and batch baselines.
if nargin<1||isempty(runs),runs=10;end;if nargin<2,opts=struct;end
modes={'y','g','n'}; algs={'APF-only','RRT*','IRRT*-only','BAIR','Batch-RRT*','BATCH-Informed-RRT*','RRT-Connect'};
results=struct('mode',{},'algorithm',{},'run',{},'success',{},'length',{},'time',{},'nodes',{});
z=0;
for im=1:numel(modes)
    env=bair_env2d(modes{im});
    for r=1:runs
        rng(20260301+100*im+r,'twister');
        % APF-only
        apfOnlyOpts=opts; apfOnlyOpts.useFallback=false;
        tic;[p,ai]=stable_apf2d(env,apfOnlyOpts);tt=toc;z=z+1;results(z)=pack(modes{im},algs{1},r,ai.success,path_length2d(p),tt,0);
        % RRT*
        tic;[p,ri]=rrtstar2d(env,opts);tt=toc;z=z+1;results(z)=pack(modes{im},algs{2},r,ri.success,ri.length,tt,ri.nodes);
        % IRRT*-only
        tic;[p,ri]=informed_rrtstar2d(env,[],opts);tt=toc;z=z+1;results(z)=pack(modes{im},algs{3},r,ri.success,ri.length,tt,ri.nodes);
        % BAIR
        tic;[p,ai]=stable_apf2d(env,opts);[p,bi]=informed_rrtstar2d(env,p,opts);tt=toc;z=z+1;results(z)=pack(modes{im},algs{4},r,bi.success,bi.length,tt,bi.nodes);
        % Batch-RRT*
        tic;[p,qi]=batch_rrtstar2d(env,opts);tt=toc;z=z+1;results(z)=pack(modes{im},algs{5},r,qi.success,qi.length,tt,qi.nodes);
        % Batch informed baseline
        tic;[p,qi]=batch_informed_rrtstar2d(env,[],opts);tt=toc;z=z+1;results(z)=pack(modes{im},algs{6},r,qi.success,qi.length,tt,qi.nodes);
        % Bidirectional RRT-Connect
        tic;[p,qi]=rrtconnect2d(env,opts);tt=toc;z=z+1;results(z)=pack(modes{im},algs{7},r,qi.success,qi.length,tt,qi.nodes);
    end
end
if isfield(opts,'outputFile') && ~isempty(opts.outputFile), writetable(struct2table(results),opts.outputFile); end
end
function s=pack(m,a,r,ok,L,t,n),s=struct('mode',m,'algorithm',a,'run',r,'success',ok,'length',L,'time',t,'nodes',n);end
function L=path_length2d(p),if size(p,1)<2,L=inf;else,L=sum(sqrt(sum(diff(p).^2,2)));end,end
