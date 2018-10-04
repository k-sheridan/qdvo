function [pixel, cov] = subPixelMatchFromQuadraticInterp(centerPixel, neighborhood)
%SUBPIXELMATCHFROMQUADRATICINTERP uses a 1 pixel radius to interpolate the
%sub pixel match. This function should be tested thoroughly.

persistent P pseudoP

if isempty(P)
    syms x y
    row = [x^2, y^2, x*y, x, y, 1];
    P = [double(subs(row, [x,y], [-1, -1]));
         double(subs(row, [x,y], [0, -1]));
         double(subs(row, [x,y], [1, -1]));
         double(subs(row, [x,y], [-1, 0]));
         double(subs(row, [x,y], [0, 0]));
         double(subs(row, [x,y], [1, 0]));
         double(subs(row, [x,y], [-1, 1]));
         double(subs(row, [x,y], [0, 1]));
         double(subs(row, [x,y], [1, 1]))]
    pseudoP = inv(P'*P)*P';
end

b = [neighborhood(1, 1:3)'; neighborhood(2, 1:3)'; neighborhood(3, 1:3)']

coeff = pseudoP*b % computes the fit coefficients.

%syms x y
%fn = coeff' * [x^2;y^2;x*y;x;y;1];
%fcontour(fn)

delta = [-coeff(4) / (2*coeff(1)); -coeff(5) / (2*coeff(2))]; % check for zero accelerations.

cov_inv = [abs(coeff(1)), -coeff(3); -coeff(3), abs(coeff(2))];

if (norm(coeff(1)) < 1e-8); delta(1) = 0; cov_inv(1, 1) = 1e-8; end
if (norm(coeff(2)) < 1e-8); delta(2) = 0; cov_inv(2, 2) = 1e-8; end

cov = inv(cov_inv);

% check the covariance matrix


%hold on
%plot3(delta(1), delta(2), 0, 'o')

pixel = centerPixel + delta;

end

