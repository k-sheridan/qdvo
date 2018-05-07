clf;
hold on;
daspect([1,1,1])
%view(70, -10);



plotCamera('Orientation', camera_pose(1:3, 1:3)', 'Location', camera_pose(1:3, 4), 'Size', 0.1);

plotCamera('Orientation', estimated_camera_pose(1:3, 1:3)', 'Location', estimated_camera_pose(1:3, 4), 'Size', 0.1, 'Color', [0,1,1])

%transform
transformed_features = camera_pose*[features; ones(1, length(features(1, :)))];
plot3(transformed_features(1, :), transformed_features(2, :), transformed_features(3, :), 'bo');

gtp = plot3(points(1, :), points(2, :), points(3, :), 'ro');

ep = plot3(estimated_points(1, :), estimated_points(2, :), estimated_points(3, :), 'go');

for index = (1:length(estimated_points(1, :)))
    covp = estimated_points_covs(1:3, (3*index - 2):(3*index));
    [vec, vals] = eig(covp);
    scale = 0.1;
    delta = [sqrt(vals(1,1))*scale * vec(1:3, 1), sqrt(vals(2,2))*scale * vec(1:3, 2), sqrt(vals(3,3))*scale * vec(1:3, 3)];
    
    p1 = estimated_points(1:3, index) + delta(1:3, 1);
    p2 = estimated_points(1:3, index) - delta(1:3, 1);
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], '-');
    
    p1 = estimated_points(1:3, index) + delta(1:3, 2);
    p2 = estimated_points(1:3, index) - delta(1:3, 2);
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], '-');
    
    p1 = estimated_points(1:3, index) + delta(1:3, 3);
    p2 = estimated_points(1:3, index) - delta(1:3, 3);
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], '-');
    
end


%camtarget(estimated_camera_pose(1:3, 4))
campos([-2, -2, -2]);

ecp = plot3(estimated_camera_pose(1, 4), estimated_camera_pose(2, 4), estimated_camera_pose(3, 4));

camlookat([gtp, ecp]);


xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')

title('Monocular Visual Inertial Odometry')


drawnow;

frame = getframe(gcf);
writeVideo(v, frame);
