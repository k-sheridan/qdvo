% this generates synthetic feature correspondences and tests the
% fundamental matrix calculator

K = [300, 0, 600; 0, 300, 600; 0, 0, 1];
invK = inv(K);
features1 = []; % random pixel vector

for x = (-1:0.1:1)
    for y = (-1:0.1:1)
        px = [x;y;1];
        features1 = [features1, px(1:2, 1)];
    end
end

R = rotx(pi/8);
t = [-0.1;0.1;0];

features2 = [];

% generate correspondences
for index = (1:length(features1(1, :)))
    bearing = [features1(1:2, index); 1];
    % generate random Z
    z =  2 + rand * 1;
    
    pt = z * bearing;
    
    ptTrans = R*pt + t;
    
    bearingTrans = ptTrans / ptTrans(3);
    
    px2 = bearingTrans;
    
    features2 = [features2, px2(1:2, 1)];
end


% compute the theoretical F
Rinv = R';
tinv = -R'*t;

a = t;
skewT = [0, -a(3), a(2); 
        a(3), 0, -a(1); 
        -a(2), a(1), 0];

F = skewT * R

% manually perturb some correspondences.
features2(1:2, 1) = features2(1:2, 1) + [0;-0.1];

% draw the two frames
scatter(features1(1, :), features1(2, :))
title('Frame 1')

figure
scatter(features2(1, :), features2(2, :))
title('Frame 2')



[F_fit, chi2avg, inliers] = computeFundamentalMatrixRANSAC(features1, features2, 20, 1e-3);

F_fit
chi2avg
inliers(1)

