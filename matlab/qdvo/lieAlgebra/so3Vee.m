function [tangent] = so3Vee(hat)
%vee operator on so3

tangent = [-hat(2, 3);hat(1, 3);-hat(1, 2)];

end

