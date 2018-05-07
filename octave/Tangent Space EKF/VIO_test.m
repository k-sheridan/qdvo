rng(1);

% generate true points
points = [];
x_scale = 1;
y_scale = 1;
z_scale = 1;
for col = (1:1:50)
    points = [points, [(rand-0.5)*x_scale; (rand-0.5)*y_scale; (rand)*z_scale + 1]];
end

depth_noise = 0.01;

estimated_points = [];
for p = points
    homo = [p(1)/p(3); p(2)/p(3); p(3)];
    new_z = p(3) + randn*depth_noise;
    
    estimated_points = [estimated_points, [homo(1)*new_z; homo(2)*new_z; new_z]];
end

dt = 0.05;

sensor_noise = (0.001)^2;

sin_angle = 0;
angle_wiggle_amp = 0;
translation_wiggle_amp = 0;

camera_twist = [0.5;0;0;0;-pi/10;0];
camera_pose = se3Exp([0;0;0;0;0;0]);

% state estimate
estimated_camera_pose = se3Exp([0;0;0;0;0;0]);
cov = diag([1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 30, 30, 30, 30, 30, 30, 100, 100, 100]);
state = zeros(15, 1);

% simulate and draw
for t = (0:dt:30)
    
    %GENERATE FAKE FEATURES
    features = [];
    for p = points
        homo = inv(camera_pose) * [p;1];
        homo = [homo(1)/homo(3) + randn*sensor_noise; homo(2)/homo(3) + randn*sensor_noise; 1];
        features = [features, homo];
    end
    
    
    %RUN OPTIMIZER
    %estimated_camera_pose(1:3, 1:3) = camera_pose(1:3, 1:3);
    
    [estimated_camera_pose, cov, state] = motion_structure_update(estimated_camera_pose, cov, state, estimated_points, features);
   
    %PLOT
    clf;
    hold on;
    daspect([1,1,1])
    view(70, -10);
    
    plotCamera('Orientation', camera_pose(1:3, 1:3)', 'Location', camera_pose(1:3, 4), 'Size', 0.1);
    
    plotCamera('Orientation', estimated_camera_pose(1:3, 1:3)', 'Location', estimated_camera_pose(1:3, 4), 'Size', 0.1, 'Color', [0,1,1])
    
    %transform 
    transformed_features = camera_pose*[features; ones(1, length(features(1, :)))];
    plot3(transformed_features(1, :), transformed_features(2, :), transformed_features(3, :), 'bo');
    
    plot3(points(1, :), points(2, :), points(3, :), 'ro');
    
    plot3(estimated_points(1, :), estimated_points(2, :), estimated_points(3, :), 'go')
    drawnow;
    
    %MOVE CAMERA
    %generate "wiggle"
    wiggle = [translation_wiggle_amp*sin(sin_angle); translation_wiggle_amp*sin(sin_angle); translation_wiggle_amp*sin(sin_angle); angle_wiggle_amp*sin(sin_angle); angle_wiggle_amp*sin(sin_angle); angle_wiggle_amp*sin(sin_angle)];
    sin_angle = sin_angle + dt;
    
    
    camera_pose = camera_pose * se3Exp(dt*(camera_twist + wiggle));
    %estimated_camera_pose = estimated_camera_pose * se3Exp(dt*(camera_twist + wiggle)); % apply odometry
    
    % apply odometry
    [cov, state, estimated_camera_pose] = gyro_update(cov, state, estimated_camera_pose, (camera_twist(4:6) + wiggle(4:6)));
    
    
    %PROCESS STATE ESTIMATE
    [state, cov, estimated_camera_pose] = process(state, cov, estimated_camera_pose, dt);
end