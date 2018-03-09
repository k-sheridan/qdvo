function [H] = unitTwistReprojectionJacobian(xyz_in_f)
%vyz_inf_f == the current position of the pixel in the current frame

rx = xyz_in_f(1);
ry = xyz_in_f(2);
rz = xyz_in_f(3);

H = [ -1/rz,     0, rx/rz^2,  (rx*ry)/rz^2, - rx^2/rz^2 - 1,  ry/rz;
     0, -1/rz, ry/rz^2, ry^2/rz^2 + 1,   -(rx*ry)/rz^2, -rx/rz];
end

