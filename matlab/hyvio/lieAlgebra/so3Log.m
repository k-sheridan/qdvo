function [phi] = so3Log(R)
% computes the ln(SO(3)) -> so(3)
% I tested this, but only trust it ~98%

theta = real(acos((trace(R) - 1)/2));

if norm(theta) < 1e-8
    phi = so3Vee(R); % assuming a small angle approximation R(phi) = I + hat(phi)
else
    phi = theta/(2*sin(theta)) * so3Vee(R - R');
end

end

