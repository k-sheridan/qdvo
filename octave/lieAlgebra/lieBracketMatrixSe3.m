function [M] = lieBracketMatrixSe3(a)

M = [so3Hat(a(4:6, 1)), so3Hat(a(1:3, 1)); zeros(3), so3Hat(a(4:6, 1))];

end

