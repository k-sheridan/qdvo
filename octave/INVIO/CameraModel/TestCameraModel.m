vignette = imread('vignette.png', 'PNG');

cm = EquidistantCameraModel([0.010171079892421483, -0.010816440029919381, 0.005942781769412756, -0.001662284667857643],...
    pi, [380.81042871360756, 380.81194179427075], [510.29465304840727, 514.3304630538506], [1024, 1024], 10000, vignette)

points = [];
for x = (-20:0.2:20)
    for y = (-20:0.2:20)
        pixel = cm.project([x;y;1]);
        if (pixel(1) >= 0 && pixel(1) <= 1024 && pixel(2) >= 0 && pixel(2) <= 1024)
            points = [points, pixel];
        end
    end
end

imshow(vignette)
hold on
scatter(points(1, :), points(2, :), 'x')
xlim([-150, 1200])
ylim([-150, 1200])
daspect([1, 1 ,1])
title('Equidistant Distortion Model')
xlabel('pixel')
ylabel('pixel')
grid on