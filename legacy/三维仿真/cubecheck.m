function  cubeFlag = cubecheck(nearp,cubeInfo,theta,fai,newp)
   %%长方体碰撞检测如果碰撞返回1
   
   cubeFlag = 0;
   s = sqrt((newp.x-nearp.x)^2+(newp.y-nearp.y)^2+(newp.z-nearp.z)^2);
   if cubeInfo.exist
       for k1 = 1 :length(cubeInfo.axisX)
           xmin = cubeInfo.axisX(k1);
           xmax = cubeInfo.axisX(k1)+ cubeInfo.lengthx(k1);
           ymin = cubeInfo.axisY(k1);
           ymax = cubeInfo.axisY(k1)+ cubeInfo.lengthy(k1);
           zmin = cubeInfo.axisZ(k1);
           zmax = cubeInfo.axisZ(k1)+ cubeInfo.lengthz(k1);
           for k2=0:0.1:s
           x = nearp.x+k2*sin(theta)*cos(fai);
           y = nearp.y+k2*sin(theta)*sin(fai);
           z = nearp.z+k2*cos(theta);
           if (x>xmin && x<xmax) && (y>ymin && y<ymax) && (z>zmin && z<zmax)
               cubeFlag = 1;
               return;
           end
           end
       end
   end

end

