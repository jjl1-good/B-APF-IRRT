%function Path= RRTTD(startPoint,axisLWH,axisStart,cubeInfo,cylinderInfo,sphereInfo,goalPoint)
%% 三维RRT算法寻找路径点
%% 变量定义
Maxiter = 5000; %最大迭代次数
iter = 0; %当前迭代次数
step = 10; %步长
count = 1;
Thr = 20 ;%范围阈值
rewire_Thr=20;

%% 初始化树
Tv(1).x = startPoint(1);
Tv(1).y = startPoint(2);
Tv(1).z = startPoint(3);
Tv(1).pre=0;
Tv(1).cos=0;

while iter<Maxiter
    iter=iter+1;
    %% 创建随机点
    if rand(1)<0.9
    randp.x=axisLWH(1)*rand(1)+axisStart(1);
    randp.y=axisLWH(2)*rand(1)+axisStart(2);
    randp.z=axisLWH(3)*rand(1)+axisStart(3);
    else
        randp.x=goalPoint(1);
        randp.y=goalPoint(2);
        randp.z=goalPoint(3);
    end
    %% 寻找最近的点
    distance = [];
    k3 = 1;
    while k3 <= length(Tv) % should not use size() function
        dx =  randp.x - Tv(k3).x;
        dy =  randp.y - Tv(k3).y;
        dz =  randp.z - Tv(k3).z;
        distance(k3) = sqrt(dx^2 + dy^2+dz^2);
        k3 = k3+1; 
    end
    
    [~,min_idx] = min(distance);
    nearp.x= Tv(min_idx).x;
    nearp.y= Tv(min_idx).y;
    nearp.z= Tv(min_idx).z;
    nearp.cos=Tv(min_idx).cos;
   %% 拓展得到新节点 
   r = distance(min_idx);
   fai = atan2(randp.y-nearp.y,randp.x-nearp.x);
   theta = acos((randp.z-nearp.z)/r);
   newp.x =  real(nearp.x+step*sin(theta)*cos(fai));
   newp.y =  real(nearp.y+step*sin(theta)*sin(fai));
   newp.z =  real(nearp.z+step*cos(theta));
   newp.cos = nearp.cos+step;
   %% 碰撞检测
   cubeFlag = cubecheck(nearp,cubeInfo,theta,fai,newp);%%长方体障碍物检测
   cylinderFlag = cylindercheck(nearp,cylinderInfo,theta,fai,newp);%%圆柱体障碍物检测
   sphereFlag = spherecheck(nearp,sphereInfo,theta,fai,newp);%%球体障碍物检测
   if cubeFlag || cylinderFlag ||sphereFlag
       continue;
   else
   %% 重连父节点,设置最近节点为临时父节点
   tempparent.x= nearp.x;
   tempparent.y= nearp.y;
   tempparent.z= nearp.z;
   temppre = min_idx;
   tempcos=newp.cos;
   
   %% 寻找范围内的点 
    rewirep = [];
    co= [];
    for k3 = 1:length(Tv) % should not use size() function
        checkp.x= Tv(k3).x;
        checkp.y= Tv(k3).y;
        checkp.z= Tv(k3).z;
        checkp.cos = Tv(k3).cos;
        dx =  newp.x - Tv(k3).x;
        dy =  newp.y - Tv(k3).y;
        dz =  newp.z - Tv(k3).z;
        co(k3) = sqrt(dx^2+dy^2+dz^2); 
        %% 碰撞检测
         cubeFlag = cubecheck(nearp,cubeInfo,theta,fai,checkp);%%长方体障碍物检测
         cylinderFlag = cylindercheck(nearp,cylinderInfo,theta,fai,checkp);%%圆柱体障碍物检测
         sphereFlag = spherecheck(nearp,sphereInfo,theta,fai,checkp);%%球体障碍物检测
         if co(k3)>rewire_Thr ||cubeFlag || cylinderFlag ||sphereFlag
            continue;
         else
             rewirep(end+1)=k3;
             if checkp.cos+co(k3)<tempcos
                  tempcos=checkp.cos+co(k3);
                  tempparent.x= checkp.x;
                  tempparent.y= checkp.y;
                  tempparent.z= checkp.z;
                  temppre = k3;
             end
         end
        
    end

       %% 纳入新节点
       count=count+1;
       Tv(count).x=newp.x;
       Tv(count).y=newp.y;
       Tv(count).z=newp.z;
       Tv(count).pre=temppre;
       Tv(count).pre=tempcos;
       line([Tv(temppre).x  newp.x],[Tv(temppre).y  newp.y],[Tv(temppre).z  newp.z],'LineWidth',1,'Color','b');
       pause(0.01);
       %% 重布线
%        for k4=1:length(rewirep)
           
       if sqrt((newp.x-goalPoint(1))^2+(newp.y-goalPoint(2))^2+(newp.z-goalPoint(3))^2)<Thr
           break;
       end
       if iter == Maxiter
           Path=[];
           disp('路径规划失败');
           return;
       end
   end

 
 
end
%% 回溯父节点
index = Tv(end).pre;
count = 1;
while Tv(index).pre~=0
    Path(count,1) = Tv(index).x;
    Path(count,2) = Tv(index).y;
    Path(count,3) = Tv(index).z;
    index = Tv(index).pre;
    count =count+1;
end
 Path(count,1) = startPoint(1);
 Path(count,2) = startPoint(2);
 Path(count,3) = startPoint(3);
 
 Path = flipud(Path);
 Path(count+1,1) = goalPoint(1);
 Path(count+1,2) = goalPoint(2);
 Path(count+1,3) = goalPoint(3);
end

