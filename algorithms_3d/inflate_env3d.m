function envOut = inflate_env3d(env, r)
%INFLATE_ENV3D 把环境中的所有障碍按半径 r 膨胀（Minkowski 和，球膨胀）。
%
%   envOut = inflate_env3d(env, 1.0)
%
% 用途：给"有足迹的机器人"做等价换算——把障碍膨胀 r、机器人当质点，
% 得到的路径与"机器人半径 r + 原障碍"严格等价（对凸障碍二者一致；
% 对 AABB/圆柱/球均为精确定义，因为这里就是按解析式膨胀的）。
%
% 膨胀规则（与 collisionChecking3D.m 的 margin 完全一致）：
%   cube     : 最小角减去 r，三边各加 2r
%   cylinder : 半径加 r，底面下移 r，高度加 2r
%   sphere   : 半径加 r
% bounds 保持不变（工作空间边界不膨胀），因此起终点仍需自行确认在自由空间内。
%
% 注意：本函数不改变默认行为——只有显式调用（或 opts.inflate > 0）时才生效，
%       论文主实验的结果不受影响。

if nargin < 2 || isempty(r) || r <= 0
    envOut = env;
    return
end
envOut = env;

loh = [env.cube.axisX(:), env.cube.axisY(:), env.cube.axisZ(:)];
if ~isempty(loh)
    envOut.cube.axisX = loh(:, 1)' - r;
    envOut.cube.axisY = loh(:, 2)' - r;
    envOut.cube.axisZ = loh(:, 3)' - r;
    envOut.cube.lengthx = env.cube.lengthx(:)' + 2 * r;
    envOut.cube.lengthy = env.cube.lengthy(:)' + 2 * r;
    envOut.cube.lengthz = env.cube.lengthz(:)' + 2 * r;
end

if ~isempty(env.cylinder.X)
    envOut.cylinder.radius = env.cylinder.radius(:)' + r;
    envOut.cylinder.Z = env.cylinder.Z(:)' - r;
    envOut.cylinder.lengthZ = env.cylinder.lengthZ(:)' + 2 * r;
end

if ~isempty(env.sphere.X)
    envOut.sphere.radius = env.sphere.radius(:)' + r;
end

% 数量（顺序）不变，仅尺寸变化；保留起终点与边界
end
