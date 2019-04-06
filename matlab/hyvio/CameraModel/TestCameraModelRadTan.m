vignette = imread('vignette.png', 'PNG');

cm = RadTanCameraModel([-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05],...
    pi/1.3, [458.654, 457.296], [367.215, 248.375], [752, 480], 10000, vignette)

points = [];
for x = (-10:0.2:10)
    for y = (-10:0.2:10)
        pixel = cm.project([x;y;2]);
        if (pixel(1) >= 0 && pixel(1) <= 752 && pixel(2) >= 0 && pixel(2) <= 480)
            points = [points, pixel];
        end
    end
end

%imshow(vignette)
hold on
scatter(points(1, :), points(2, :), 'x')
%xlim([-150, 1200])
%ylim([-150, 1200])
daspect([1, 1 ,1])
title('RadTan Distortion Model: 10mX10m Grid 1 meter from camera')
xlabel('pixel')
ylabel('pixel')
grid on


% test project jacobians
pos = [0.1;-0.2;2];
J = zeros(2, 3);
dx = zeros(3, 1);
delta = 1e-4;

for idx = (1:3)
    dx(idx) = dx(idx) + delta;
    pxHigh = cm.project(pos+dx);
    dx(idx) = dx(idx) - 2*delta;
    pxLow = cm.project(pos+dx);
    dx(idx) = dx(idx) + delta;
    J(1:2, idx) = (pxHigh-pxLow)/(2*delta);
end

[px,jac] = cm.project(pos);

jac * [1/pos(3), 0, -pos(1)/pos(3)^2; 0, 1/pos(3), -pos(2)/pos(3)^2]
J
