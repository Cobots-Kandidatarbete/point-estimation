function [tform, ptCloudTruth,ptCloud_GR, tform_global] = estimatePose(PointCloud,gr)

% Denoise the point cloud
ptCloud = pcdenoise(PointCloud);

% Removes points that are not a part of the box i.e. table, floor etc.
ptCloud = RemoveXYplane(ptCloud);

% Downsample point cloud to 500 points, this improves the performance of the
% registration algorithm
ptCloudDS = pcdownsample(ptCloud, 'random', 500/ptCloud.Count);

% The point cloud generated from the CAD-model has the unit mm and is
% therefore rescaled by a factor 1000 to get the unit m
TruthPoints = pcread('box_750.ply').Location/1000;
ptCloudTruth = pointCloud(TruthPoints);

% Downsamle CAD-point cloud
ptCloudTruthDS_original = pcdownsample(ptCloudTruth, 'random', 500/ptCloudTruth.Count);

% Generate a rotated version of the point cloud generated from CAD
[ptCloudTruthDS_rot, rot] = rot_pc_z(ptCloudTruthDS_original, pi/2);
% Cell containing boxes in two angles
ptCloudTruthDS = {ptCloudTruthDS_original, ptCloudTruthDS_rot};

rmse = zeros(1,2);

% The loop uses CPD/NDT to get an initial transform, which is then improved by
% ICP. Index i loops over different initial rotations to avoid local minimum
for i = 1:length(ptCloudTruthDS)
    if gr == 'ndt'
        [tform_global{i}, ~, ~] = pcregisterndt(ptCloudTruthDS{i}, ptCloudDS, 0.05, 'MaxIterations',1000, 'Tolerance', [1e-2, 1e-7], 'OutlierRatio',0.3);
        ptCloud_GR{i} = pctransform(ptCloudTruth,tform_global{i});
        [tform{i}, ~, rmse(i)] = pcregistericp(ptCloudTruthDS{i}, ptCloudDS,'MaxIterations',1000, 'Tolerance', [1e-2, 1e-7], 'InlierRatio', 0.7, 'InitialTransform', tform_global{i});

    elseif gr == 'cpd'
        [tform_global{i}, ~, ~] = pcregistercpd(ptCloudTruthDS{i}, ptCloudDS,'Transform', 'Rigid','MaxIterations',1000, 'Tolerance', 1e-5,'OutlierRatio',0.3);
        ptCloud_GR{i} = pctransform(ptCloudTruth,tform_global{i});
        [tform{i}, ~, rmse(i)] = pcregistericp(ptCloudTruthDS{i}, ptCloudDS,'MaxIterations',1000, 'Tolerance', [1e-2, 1e-7], 'InlierRatio', 0.7, 'InitialTransform', tform_global{i});

    end
end
% Save the tranform that generates the lowest rmse value
bestIdx = find(rmse == min(rmse));
if bestIdx == 1
    tform = tform{bestIdx};
    tform_global = tform_global{bestIdx};
    ptCloud_GR = ptCloud_GR{bestIdx};
else
    % The initial transform must be added if the bestIDX = 2
    tform = affine3d(rot.T*tform{bestIdx}.T);
    tform = rigid3d(tform.T(1:3,1:3), tform.T(4,1:3));
    
    tform_global = affine3d(rot.T*tform_global{bestIdx}.T);
    tform_global = rigid3d(tform_global.T(1:3,1:3), tform_global.T(4,1:3));
    ptCloud_GR = pctransform(ptCloudTruth,tform_global);
end
ptCloudTruth = pctransform(ptCloudTruth,tform);

end

