function [adj] = se3Adjoint(T)
    
    adj(1:3, 1:3) = T(1:3, 1:3);
    
    adj(4:6, 4:6) = T(1:3, 1:3);
    
    adj(1:3, 4:6) = so3Hat(T(1:3, 4))*T(1:3, 1:3);
    
    adj(4:6, 1:3) = zeros(3);

end

