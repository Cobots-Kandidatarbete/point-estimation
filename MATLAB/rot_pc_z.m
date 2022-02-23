function [pc_rot, T] = rot_pc_z(pc,th)

Rz = [cos(th) -sin(th) 0;
      sin(th) cos(th) 0;
      0 0 1];
H = [Rz zeros(3,1);
     zeros(1,3) 1];
T = affine3d(H);
pc_rot = pctransform(pc, T);

end