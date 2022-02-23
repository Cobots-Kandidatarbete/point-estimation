%%%%%%%%%% Main script for running the pose estimation algorithm %%%%%%%%%%
clc
clear all
clf;
close all 


% Load point cloud, input the point cloud scan
ptCloud = pcread('realsense_testcase2.ply');


% Rotate point cloud to xy-plane (might need to be changed)
pc_rot = rotate_pc(ptCloud);
% Crop point cloud around box
pc_box = pc_crop(pc_rot);

% Plot new PC
figure(2)
pcshow(pc_rot)
hold on
pcshow(pc_box.Location,'r')
xlabel('x')
ylabel('y')
zlabel('z')

figure(3)
hold on
pcshow(pc_box)
xlabel('x')
ylabel('y')
zlabel('z')

% Function that aligns the point clouds
[tform, ptCloudTruth, ptCloudMid, tform_Mid] = estimatePose(pc_box,'cpd');

figure(4)
hold on
pcshow(pc_box.Location,'y')
pcshow(ptCloudMid.Location,'b')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
title('Only CPD')

figure(5)
hold on
pcshow(pc_box.Location,'y')
pcshow(ptCloudTruth.Location,'b')
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
title('CPD and ICP')