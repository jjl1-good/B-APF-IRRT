function env = bair_env2d(mode)
%BAIR_ENV2D Fixed benchmark environments used by the original manuscript.
% The obstacle, boundary and start/goal values are intentionally unchanged.
if nargin < 1, mode = 'g'; end
mode = lower(char(mode));
env.resolution = 1;
switch mode
    case 'g'
        env.bounds = [0 300 0 300];
        env.square = [95 80 20 40; 85 145 20 40; 135 115 30 30];
        env.round = [175 175 12; 205 175 12; 145 185 12];
        env.start = [76 76]; env.goal = [220 220];
    case 'y'
        env.bounds = [0 150 0 150];
        env.square = [20 5 20 40; 10 70 20 40; 60 40 30 30];
        env.round = [100 100 12; 130 100 12; 70 110 12];
        env.start = [1 1]; env.goal = [145 145];
    case 'n'
        env.bounds = [0 150 0 150];
        env.square = [20 5 20 40; 10 46 20 40; 60 40 30 30];
        env.round = [95 100 12; 130 100 12; 70 110 12];
        env.start = [1 1]; env.goal = [145 145];
    otherwise
        error('Unknown BAIR environment: %s', mode);
end
% Add the same 3-unit walls used by the original implementations.
b = env.bounds; L=b(1); R=b(2); D=b(3); U=b(4);
env.walls = [L-3 D 3 U-D+3; L-3 D-3 R-L+3 3; R D-3 3 U-D+3; L U R-L+3 3];
env.squareAll = [env.square; env.walls];
env.name = mode;
end
