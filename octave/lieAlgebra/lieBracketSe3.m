function [c] = lieBracketSe3(a, b)
%order vx vy vz wx wy wz

c(1:3, 1) = so3Hat(a(4:6, 1)) * b(1:3, 1) + so3Hat(a(1:3, 1)) * b(4:6, 1);
c(4:6, 1) = so3Hat(a(4:6, 1)) * b(4:6, 1);
end

