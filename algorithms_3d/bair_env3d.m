function env = bair_env3d(mode)
%BAIR_ENV3D 三维基准环境定义（含新增真三维场景）。
%
% 保留原有两个环境（general / narrow，结构不变，保证可复现性），并新增：
%   'suspended' : 悬空球阵 + 高架薄板平台（无二维对应物）
%   'ring'      : 环形柱廊 + 中央高柱（上方开口，需三维螺旋绕行）
%   'overhang'  : 悬挑板与悬空球组合（上下双通道）
% 字段结构保持与原实现一致（cube/cylinder/sphere），使
% collisionChecking3D / stable_apf3d / 各规划器无需特判。

if nargin < 1, mode = 'general'; end
mode = lower(char(mode));

switch mode
    case {'narrow', 'n'}
        env.bounds = [0 150 0 150 0 150];
        % 原仓库终点 [150 150 150] 恰好落在球面障碍（150,150,130,r=20）表面上，
        % 在任何考虑安全裕度的检查下均为非法位形；此处移至最近的自由点（净空 7.8）。
        env.start = [1 1 1]; env.goal = [135 135 148]; env.name = 'narrow';
        env = embed_legacy_obstacles(env);

    case {'general', 'wide', 'g'}
        % 原仓库 general / wide 使用同一地图（保持不变）。
        env.bounds = [0 200 0 200 0 150];
        env.start = [26 26 1]; env.goal = [175 175 150]; env.name = mode;
        env = embed_legacy_obstacles(env);

    case 'suspended'
        env.bounds = [0 200 0 200 0 150];
        env.start = [20 20 10]; env.goal = [180 180 140]; env.name = 'suspended';
        % 悬空球阵（错位两排，高低不同）
        env.sphere.X = [70 70 70 130 130 130];
        env.sphere.Y = [40 100 160 60 120 180];
        env.sphere.Z = [60 95 70 100 60 95];
        env.sphere.radius = [18 20 18 20 18 20];
        % 高架薄板（悬空平台，横跨起终点直连方向，必须三维绕行）
        env.cube.axisX = [85 20]; env.cube.axisY = [85 140]; env.cube.axisZ = [70 60];
        env.cube.lengthx = [40 40]; env.cube.lengthy = [60 40]; env.cube.lengthz = [10 8];
        env.cylinder.X = []; env.cylinder.Y = []; env.cylinder.Z = [];
        env.cylinder.radius = []; env.cylinder.lengthZ = [];

    case 'ring'
        env.bounds = [0 150 0 150 0 150];
        env.start = [15 75 15]; env.goal = [135 75 135]; env.name = 'ring';
        % 环形柱廊（45°/90° 两柱高度受限，形成上方开口）
        ang = (0:45:315) * pi / 180;
        Rring = 45;
        env.cylinder.X = 75 + Rring * cos(ang);
        env.cylinder.Y = 75 + Rring * sin(ang);
        env.cylinder.Z = zeros(1, 8);
        env.cylinder.radius = 7.5 * ones(1, 8);
        env.cylinder.lengthZ = [150 95 95 150 150 150 150 150];
        % 中央高柱（阻断直线穿越）
        env.cylinder.X(end + 1) = 75; env.cylinder.Y(end + 1) = 75;
        env.cylinder.Z(end + 1) = 0; env.cylinder.radius(end + 1) = 15;
        env.cylinder.lengthZ(end + 1) = 150;
        env.cube.axisX = []; env.cube.axisY = []; env.cube.axisZ = [];
        env.cube.lengthx = []; env.cube.lengthy = []; env.cube.lengthz = [];
        env.sphere.X = []; env.sphere.Y = []; env.sphere.Z = []; env.sphere.radius = [];

    case 'overhang'
        env.bounds = [0 200 0 200 0 120];
        env.start = [20 180 10]; env.goal = [180 20 110]; env.name = 'overhang';
        % 悬挑板（不同高度，构成上下双通道）
        env.cube.axisX = [80 130 40]; env.cube.axisY = [60 20 100]; env.cube.axisZ = [80 40 95];
        env.cube.lengthx = [60 50 80]; env.cube.lengthy = [80 40 40]; env.cube.lengthz = [10 8 8];
        % 悬空球（位于板下与开口附近，迫使三维绕行）
        env.sphere.X = [110 90 60]; env.sphere.Y = [100 40 140]; env.sphere.Z = [45 70 30];
        env.sphere.radius = [18 16 14];
        env.cylinder.X = []; env.cylinder.Y = []; env.cylinder.Z = [];
        env.cylinder.radius = []; env.cylinder.lengthZ = [];

    case {'general_v2', 'g2'}
        % 三维“一般”场景（重设计）：把二维一般环境（150 框）的障碍布局按
        %   p' = 26 + (p - 1) * 149/144
        % 映射到 200x200x150 框内（起终点沿用 legacy 值），并竖向挤出为
        % 三种不同高度的盒体 / 柱体，构成沿起终点路线依次展开的绕行结构。
        % 动机见交付说明：原 legacy general 与 narrow 共用同一组障碍物，起点
        % (26,26,1) 夹在两个高 30 的盒体之间形成凹角，反应式预规划反复进入
        % 逃逸状态（escapes 5 / flips 11），种子长度达近最优的 1.27 倍。
        env.bounds = [0 200 0 200 0 150];
        env.start = [26 26 1]; env.goal = [175 175 150]; env.name = 'general_v2';
        env.cube.axisX = [45.66 35.31 87.05];
        env.cube.axisY = [30.14 97.40 66.35];
        env.cube.axisZ = [0 0 0];
        env.cube.lengthx = [20.69 20.69 31.04];
        env.cube.lengthy = [41.39 41.39 31.04];
        env.cube.lengthz = [150 110 150];
        env.cylinder.X = [128.47 159.51 97.40];
        env.cylinder.Y = [128.47 128.47 138.82];
        env.cylinder.Z = [0 0 0];
        env.cylinder.radius = [12.42 12.42 12.42];
        env.cylinder.lengthZ = [150 130 90];
        env.sphere.X = []; env.sphere.Y = []; env.sphere.Z = []; env.sphere.radius = [];

    otherwise
        error('Unknown BAIR 3-D environment: %s', mode);
end
end

function env = embed_legacy_obstacles(env)
% 原实现中 general 与 narrow 使用同一组障碍物参数（保持不变）。
env.cube.axisX = [50 20 100]; env.cube.axisY = [25 45 80]; env.cube.axisZ = [0 0 0];
env.cube.lengthx = [25 20 40]; env.cube.lengthy = [25 20 15]; env.cube.lengthz = [30 30 130];
env.cylinder.X = [75 75 120]; env.cylinder.Y = [75 125 60]; env.cylinder.Z = [0 0 0];
env.cylinder.radius = [25 20 20]; env.cylinder.lengthZ = [100 120 120];
env.sphere.X = [150 115]; env.sphere.Y = [150 115]; env.sphere.Z = [130 110];
env.sphere.radius = [20 20];
end
