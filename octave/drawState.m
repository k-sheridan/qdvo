function [] = drawState(T,cov)
clf;
hold on;
unitx = T(1:2, 1:2)*[1;0];
quiver(T(1, 4), T(2, 4), unitx(1), unitx(2));
unity = T(1:2, 1:2)*[0;1];
quiver(T(1, 4), T(2, 4), unity(1), unity(2));

R = T(1:2, 1:2);
plot_gaussian_ellipsoid(R*cov(1:2, 1:2)*R', T(1:2, 4));

drawnow;
end

