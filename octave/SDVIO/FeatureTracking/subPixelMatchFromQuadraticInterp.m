function [pixel, cov] = subPixelMatchFromQuadraticInterp3(centerPixel, neighborhood)
%SUBPIXELMATCHFROMQUADRATICINTERP uses a 1 pixel radius to interpolate the
%sub pixel match. This function should be tested thoroughly.

radius = 2; % a setting used to compute the pseudoP. like a kernel size.

persistent P pseudoP

if isempty(P)
    syms x y
    row = [x^2, y^2, x*y, x, y, 1];
    
    % generate test points
    testPoints = [];
    for xi = (-radius:1:radius)
        for yi = (-radius:1:radius)
            testPoints = [testPoints, [xi; yi]];
        end
    end
    
    P = [];
    % create the P dynamically
    for pt = testPoints
        P = [P; double(subs(row, [x;y], pt))];
    end
    P
    pseudoP = inv(P'*P)*P';
end

b = reshape(neighborhood', (2*radius+1)^2, 1);

coeff = pseudoP*b % computes the fit coefficients.

delta = [-coeff(4) / (2*coeff(1)); -coeff(5) / (2*coeff(2))]; % check for zero accelerations.

cov_inv = [abs(coeff(1)), -coeff(3); -coeff(3), abs(coeff(2))];

if (norm(coeff(1)) < 1e-8); delta(1) = 0; cov_inv(1, 1) = 1e-8; warning('extremely high x uncertainty'); end
if (norm(coeff(2)) < 1e-8); delta(2) = 0; cov_inv(2, 2) = 1e-8; warning('extremely high y uncertainty'); end

cov = inv(cov_inv);
pixel = centerPixel + delta;

% Check the covariance matrix
assert(min(eig(cov)) >= 0);

% Optional drawing.
%hold on
%image(-radius, -radius, neighborhood', 'CDataMapping','scaled');
%colormap gray
%colorbar
%caxis([-1, 1])
%syms x y
%fn = coeff' * [(x)^2;(y)^2;(x)*(y);(x);(y);1];
%fcontour(fn, [-radius-1, radius+1], 'LineColor', [0,1,0])
%hold on
%plot3(delta(1), delta(2), 0, '+')
%xlabel('x')
%ylabel('y')

end

