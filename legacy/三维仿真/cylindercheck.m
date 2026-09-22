        function  cylinderFlag = cylindercheck(nearp,cylinderInfo,theta,fai,newp)
   %%长方体碰撞检测如果碰撞返回
   
   cylinderFlag = 0;
   s = sqrt((newp.x-nearp.x)^2+(newp.y-nearp.y)^2+(newp.z-nearp.z)^2);
   if cylinderInfo.exist
       for k1 = 1 :length(cylinderInfo.X)
           zmin = cylinderInfo.Z(k1);
           zmax = cylinderInfo.Z(k1)+ cylinderInfo.lengthZ(k1);
           for k2=0:0.1:s
           x =  nearp.x+k2*sin(theta)*cos(fai);
               y =  nearp.y+k2*sin(theta)*sin(fai);
           z =  nearp.z+k2*cos(theta);
           if sqrt((x-cylinderInfo.X(k1))^2+(y-cylinderInfo.Y(k1))^2)<cylinderInfo.radius(k1) && z>zmin && z<zmax
               cylinderFlag = 1;
               return;
           end
           end
       end
   end

end


