function drawsphere(sphereInfo,colorMatSphere,pellucidity)
 
if sphereInfo.exist
    for k1 = 1:size(sphereInfo.X,2)
        xCoor = sphereInfo.X(k1);
        yCoor = sphereInfo.Y(k1);
        zCoor = sphereInfo.Z(k1);
        radius = sphereInfo.radius(k1);
        
        [x,y,z] = sphere(50);
        mesh(x*radius+xCoor,y*radius+yCoor,z*radius+zCoor,'FaceColor',colorMatSphere,'EdgeColor','none','FaceAlpha',pellucidity);
        
    end 
    
end
 
end
