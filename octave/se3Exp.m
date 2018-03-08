function [T] = se3Exp(se3)
%exponential map of se3 closed form

w = se3(4:6);
v = se3(1:3);


if(norm(w) < 1e-7)
    T = eye(4, 4) + se3Hat(se3);
else
    w_hat = so3Hat(w);
    
    R = eye(3, 3) + w_hat/norm(w) * sin(norm(w)) + (w_hat/norm(w))^2 * (1 - cos(norm(w)));
    
    t = ((eye(3, 3) - R) * w_hat*v + w*w'*v) / norm(w);
    
    T = [R, t; zeros(1, 3), 1];
    
end



end

