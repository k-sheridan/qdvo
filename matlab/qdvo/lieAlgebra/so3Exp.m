function [R] = so3Exp(tangent)

theta = norm(tangent);

if theta < 1e-16
    hat = so3Hat(tangent);
    R = eye(3) + hat + hat^2/2;
else
    K = so3Hat(tangent/theta);
    R = eye(3) + sin(theta)*K + (1 - cos(theta))*K^2;
end

end

