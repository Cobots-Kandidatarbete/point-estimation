function [rotated_pc] = rotate_pc(unrotated_pc)
    model = pcfitplane(unrotated_pc,0.02);
    u = model.Normal;
    u = u./norm(u);
    v = [0,0,1];
    r = vrrotvec(u,v);
    M = makehgtform('axisrotate',r(1:3),-r(end));
    tform = rigid3d(M(1:3,1:3),M(end,1:3));
    rotated_pc = pctransform(unrotated_pc,tform);
    if r(end) > 1.9
        rotated_pc = rot_pc_xy(rotated_pc);
    end
end

