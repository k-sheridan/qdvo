function [result] = bakercampbellhausdorffSe3(x,y)
%BAKERCAMPBELLHAUSDORFF approximately ln(e^(a)*e^(b))

lbx = lieBracketMatrixSe3(x);
lby = lieBracketMatrixSe3(y);

result = x + y + 1/2 * (lbx * y);

%higher order
result = result + 1/12 * ((lbx * lbx * y) + (lby * lby * x));
%result = result - 1/24 * (lby * lbx * lbx * y);
end

