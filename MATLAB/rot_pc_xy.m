function pc_rot = rot_pc_xy(pc)
% Rotate PC to x-y-plane
th = -pi;
Rx = [1 0 0;
      0 cos(th) -sin(th);
      0 sin(th) cos(th)];
H = [Rx zeros(3,1);
     zeros(1,3) 1];
T = affine3d(H);
pc_rot = pctransform(pc, T);


end