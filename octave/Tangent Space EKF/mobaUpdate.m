function [new_camera_transform] = mobaUpdate(prior_camera_transform, prior_tangent_space_uncertainty, points, features)
% run a weighted least squares update to estimate camera motion

% points: [x1, x2; y1, y2; z1, z2] etc
% features: [u1, u2; v1, v2; 1, 1] etc

ITERATION = 10;

new_camera_transform = prior_camera_transform;


P = prior_tangent_space_uncertainty;

for it = (1:ITERATION)
    
    A = zeros(6, 6);
    b = zeros(6, 1);
    
    chi = 0;
    last_chi = 1e12;
    
    for index = 1:1:length(points(1, :))
        H = twist2PixelDelta(points(1:3, index));
        
        proj = (new_camera_transform) \ [points(1:3, index);1];
        
        e = features(1:2, index) - [proj(1)/proj(3); proj(2)/proj(3)];
        
        chi = chi + norm(e);
        
        huber = huberLossFn(norm(e));
        
        A = A + H'*H*huber;
        b = b + H'*e*huber;
        
    end
    
    chi = chi / length(points(1, :));
    
    if (chi > last_chi)
        disp('Breaking because chi increased')
        break;
    end
    
    dx = (inv(P) + A) \ (b);
    %dx = A\b;
    
    new_camera_transform = new_camera_transform * se3Exp(dx);
    
end


end

