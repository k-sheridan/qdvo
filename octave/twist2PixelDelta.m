function [H] = twist2PixelDelta(pos)

rx = pos(1);
ry = pos(2);
rz = pos(3);

H =  [ -1/rz,     0, rx/rz^2,  (rx*ry)/rz^2, - rx^2/rz^2 - 1,  ry/rz;
     0, -1/rz, ry/rz^2, ry^2/rz^2 + 1,   -(rx*ry)/rz^2, -rx/rz];
end

