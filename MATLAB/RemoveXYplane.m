function newCloud = RemoveXYplane(inputCloud)
    
    [Plane, inliers, outliers] = pcfitplane(inputCloud,0.01);
    
    inlierCloud = select(inputCloud,inliers);
    outlierCloud = select(inputCloud,outliers);
    lowerBound = inlierCloud.ZLimits(1);
    
    points = outlierCloud.Location;
    filteredPointsIdx = find(points(:,3) > lowerBound);
    
    newCloud = select(outlierCloud, filteredPointsIdx);
        
end