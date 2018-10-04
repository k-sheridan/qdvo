function [pixel, cov] = subPixelMatchFromQuadraticInterp(centerPixel, neighborhood)
%SUBPIXELMATCHFROMQUADRATICINTERP uses a 1 pixel radius to interpolate the
%sub pixel match. This function should be tested thoroughly.

persistent P pseudoP

if isempty(P)
    P = [1 1 1 -1 -1 1; 0 1 0 0 -1 1; 1 1 -1 1 -1 1; 1 0 0 -1 0 1; 0 0 0 0 0 1; 1 0 0 1 0 1; 1 1 -1 -1 1 1; 0 1 0 0 1 1; 1 1 1 1 1 1];
    pseudoP = inv(P'*P)*P';
end

b = [neighborhood(1, 1:3)'; neighborhood(2, 1:3)'; neighborhood(3, 1:3)'];

coeff = pseudoP*b % computes the fit coefficients.

syms x y
fn = coeff' * [x^2;y^2;x*y;x;y;1];

fsurf(fn)

pixel = centerPixel;

%TODO check the covariance matrix
cov = inv(-[coeff(1), coeff(3); coeff(3), coeff(2)]);

end

