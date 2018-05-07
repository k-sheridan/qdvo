function [H] = dr2PixelDelta(xyz_f)

H = [1/xyz_f(3), 0, -(1/xyz_f(3))^2 * xyz_f(1);
    0, 1/xyz_f(3), -(1/xyz_f(3))^2 * xyz_f(2)];

end

