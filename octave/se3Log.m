function [tangent] = se3Log(T)
%log of transform to get se3
tangent = zeros(6,1);

phi = acos((trace(T(1:3, 1:3)) - 1)/2);

tangent(4:6,1) = so3Vee(phi/(2*sin(phi))*(T(1:3, 1:3)-T(1:3, 1:3)'));

A_inv = eye(3) - 0.5*so3Hat(tangent(4:6,1));

w_norm = norm(tangent(4:6,1));

A_inv = A_inv + ((2*sin(w_norm) - w_norm*(1 + cos(w_norm))) / (2*w_norm^2*sin(w_norm)))*so3Hat(tangent(4:6,1))*so3Hat(tangent(4:6,1));

tangent(1:3, 1) = A_inv * T(1:3, 4);

end

