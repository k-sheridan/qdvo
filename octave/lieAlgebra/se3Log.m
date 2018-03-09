function [tangent] = se3Log(T)
%log of transform to get se3

hat = zeros(4);

for n = (1:100)
    hat = hat + (-1)^(n+1) * (T - eye(4))^n/n;
end

tangent = se3Vee(hat);

end

