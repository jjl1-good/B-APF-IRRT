function drawCube(cubeInfo,colorMatCube,pellucidity)
%%»­³€·½Με
if cubeInfo.exist
    for k1 = 1:size(cubeInfo.axisX,2)
        plotcube([cubeInfo.lengthx(k1) cubeInfo.lengthy(k1) cubeInfo.lengthz(k1)],[cubeInfo.axisX(k1) cubeInfo.axisY(k1) cubeInfo.axisZ(k1)],pellucidity,colorMatCube);
    end
end

end

