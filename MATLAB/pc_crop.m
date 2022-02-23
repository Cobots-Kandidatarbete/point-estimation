function pc_box = pc_crop(pc)

% Extract points of pc
pts = pc.Location';

% Plot map for selection of box
fig = figure(10);
axis equal
hold on
grid on
xlabel('x')
ylabel('y')
zlabel('z')
scatter3(pts(1,:),pts(2,:),pts(3,:),10,pts(3,:),'filled')
colormap(jet)

% Draw rectangle on box
r1 = drawrectangle('Label','Box','Color',[0 0 0],'Rotatable',1);
%Wait for drawing to finish
wait(r1)

% Select points within ROI and save to new PC
roi = [r1.Position(1) r1.Position(1)+r1.Position(3) r1.Position(2) r1.Position(2)+r1.Position(4) -10 10];
indices = findPointsInROI(pc,roi);
pc_box = select(pc,indices);
% Find max z in cropped box and crop again based on box dimensions (known)
max_z = max(pc_box.Location(:,3));
min_x = min(pc_box.Location(:,1));
max_x = max(pc_box.Location(:,1));
min_y = min(pc_box.Location(:,2));
max_y = max(pc_box.Location(:,2));

close(fig)

end