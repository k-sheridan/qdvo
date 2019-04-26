vignette = imread('vignette.png', 'PNG');

cm = RadTanCameraModel([-0.297439888, 0.081949, -0.00005212461326460, 0.00009844260968483],...
    pi/1.1, [441.998, 441.668], [385.60887400782206669, 232.65626547906052224], [752, 480], 10000, vignette)

points = [];
for x = (-10:0.2:10)
    for y = (-10:0.2:10)
        try
            pixel = cm.project([x;y;2]);
        catch
            continue;
        end
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
