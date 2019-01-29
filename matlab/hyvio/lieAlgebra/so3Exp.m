function [R] = so3Exp(tangent)

hat = so3Hat(tangent);

R = eye(3);

for n = (1:100)
    R = R + hat^n/factorial(n);
end

end

