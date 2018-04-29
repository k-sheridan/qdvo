rng(1);

% generate points
points = [];
x_scale = 2;
y_scale = 2;
z_scale = 5;
for col = (1:1:50)
    points = [points, [(rand-0.5)*x_scale; (rand-0.5)*y_scale; (rand)*z_scale + 1]];
end

dt = 0.1;

sensor_noise = 0.0;

camera_twist = [0;0.1;-0.5;0;pi/16;0];
camera_pose = se3Exp([0;0;0;0;0;0]);

estimated_camera_pose = se3Exp([0;0;0;0;0;0]);
tangent_space_uncertainty = diag([1, 1, 1, 1, 1, 1]);

% simulate and draw
for t = (0:dt:2)
    
    %GENERATE FAKE FEATURES
    features = [];
    for p = points
        homo = inv(camera_pose) * [p;1];
        homo = [homo(1)/homo(3) + randn*sensor_noise; homo(2)/homo(3) + randn*sensor_noise; 1];
        features = [features, homo];
    end
    
    
    %RUN OPTIMIZER
    [estimated_camera_pose] = mobaUpdate(estimated_camera_pose, tangent_space_uncertainty, points, features);
   
    %PLOT
    clf;
    hold on;
    daspect([1,1,1])
    
    plotCamera('Orientation', camera_pose(1:3, 1:3)', 'Location', camera_pose(1:3, 4), 'Size', 0.1);
    
    plotCamera('Orientation', estimated_camera_pose(1:3, 1:3)', 'Location', estimated_camera_pose(1:3, 4), 'Size', 0.1, 'Color', [0,1,1])
    
    %transform 
    transformed_features = camera_pose*[features; ones(1, length(features(1, :)))];
    plot3(transformed_features(1, :), transformed_features(2, :), transformed_features(3, :), 'o');
    
    plot3(points(1, :), points(2, :), points(3, :), 'o');
    drawnow;
    
    % move camera
    camera_pose = camera_pose * se3Exp(dt*camera_twist);
end