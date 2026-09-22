function  sphereFlag = spherecheck( nearp,sphereInfo,theta,fai,newp)
   %%长方体碰撞检测如果碰撞返回1
   
   sphereFlag = 0;
   s = sqrt((newp.x-nearp.x)^2+(newp.y-nearp.y)^2+(newp.z-nearp.z)^2);
   if sphereInfo.exist
       for k1 = 1 :length(sphereInfo.X)
           for k2=0:0.1:s
           x =  nearp.x+k2*sin(theta)*cos(fai);
           y =  nearp.y+k2*sin(theta)*sin(fai);
           z =  nearp.z+k2*cos(theta);
           if (sqrt((x-sphereInfo.X(k1))^2+(y-sphereInfo.Y(k1))^2+(z-sphereInfo.Z(k1))^2)<sphereInfo.radius(k1))
               sphereFlag = 1;
               return;
           end
           end
       end
   end

end


