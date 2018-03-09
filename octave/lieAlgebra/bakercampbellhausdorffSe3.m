function [result] = bakercampbellhausdorffSe3(a,b)
%BAKERCAMPBELLHAUSDORFF approximately ln(e^(a)*e^(b))

lbab = lieBracketSe3(a, b);

result = a + b + 1/2 * lbab + 1/12 * lieBracketSe3(a, lbab) - 1/12 * lieBracketSe3(b, lbab);
end

