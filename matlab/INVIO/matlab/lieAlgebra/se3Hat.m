function [hat] = se3Hat(se3)
% se(3) hat operation linear; angular

vx = se3(1);
vy = se3(2);
vz = se3(3);
wx = se3(4);
wy = se3(5);
wz = se3(6);

hat = [0, -wz, wy, vx;
    wz, 0, -wx, vy;
    -wy, wx, 0, vz;
    0, 0, 0, 0];
end

