function [tangent] = se3Vee(hat)
%vee operator on so3

tangent(1:3, 1) = [hat(1, 4);hat(2, 4);hat(3, 4)]
tangent(4:6, 1) = [-hat(2, 3);hat(1, 3);-hat(1, 2)];

end

