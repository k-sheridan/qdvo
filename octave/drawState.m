function [] = drawState(T,cov)
clf;
hold on;
unitx = T(1:2, 1:2)*[1;0];
quiver(T(1, 4), T(2, 4), unitx(1), unitx(2));
unity = T(1:2, 1:2)*[0;1];
quiver(T(1, 4), T(2, 4), unity(1), unity(2));

plot_gaussian_ellipsoid(cov(1:2, 1:2), T(1:2, 4));

drawnow;
end

