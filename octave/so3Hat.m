function [hat] = so3Hat(so3)
%so3 hat

wx = so3(1);
wy = so3(2);
wz = so3(3);

hat = [0, -wz, wy;
    wz, 0, -wx;
    -wy, wx, 0];

end

