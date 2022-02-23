function [Eul] = Rotation2Euler(Rot)
th_x = atan2d(Rot(3,2),Rot(3,3));
th_y = atan2d(-Rot(3,1), sqrt(Rot(3,2)^2 + Rot(3,3)^2));
th_z = atan2d(Rot(2,1),Rot(1,1));

Eul = [th_x, th_y, th_z];
end

