function setup_bair_paths
%SETUP_BAIR_PATHS Add the experiment entry and shared module directories.
entryDir=fileparts(mfilename('fullpath')); rootDir=fileparts(entryDir);
addpath(entryDir); addpath(fullfile(rootDir,'algorithms_2d')); addpath(fullfile(rootDir,'algorithms_3d'));
fprintf('BAIR comparison paths added. Root: %s\n',rootDir);
end
