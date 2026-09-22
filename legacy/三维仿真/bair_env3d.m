function env = bair_env3d(mode)
%BAIR_ENV3D Fixed 3-D benchmark configurations from the original project.
if nargin<1, mode='general'; end
mode=lower(char(mode));
if strcmp(mode,'narrow') || strcmp(mode,'n')
    env.bounds=[0 150 0 150 0 150]; env.start=[1 1 1]; env.goal=[150 150 150]; env.name='narrow';
else
    % The repository's existing general and wide 3-D files both use this map.
    env.bounds=[0 200 0 200 0 150]; env.start=[26 26 1]; env.goal=[175 175 150]; env.name=mode;
end
env.cube.axisX=[50 20 100]; env.cube.axisY=[25 45 80]; env.cube.axisZ=[0 0 0];
env.cube.lengthx=[25 20 40]; env.cube.lengthy=[25 20 15]; env.cube.lengthz=[30 30 130];
env.cylinder.X=[75 75 120]; env.cylinder.Y=[75 125 60]; env.cylinder.Z=[0 0 0];
env.cylinder.radius=[25 20 20]; env.cylinder.lengthZ=[100 120 120];
env.sphere.X=[150 115]; env.sphere.Y=[150 115]; env.sphere.Z=[130 110]; env.sphere.radius=[20 20];
end
