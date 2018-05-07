function [new_camera_transform, posterior_cov, posterior_state] = motion_structure_update(prior_camera_transform, prior_tangent_space_uncertainty, prior_state, points, points_covs, features)
% run a weighted least squares update to estimate camera motion

% points: [x1, x2; y1, y2; z1, z2] etc
% features: [u1, u2; v1, v2; 1, 1] etc

ITERATION = 10;

new_camera_transform = prior_camera_transform;

posterior_cov = prior_tangent_space_uncertainty;
posterior_state = prior_state;

bearing_variance = 0.01^2
R_inv = diag([1/bearing_variance, 1/bearing_variance]);

for it = (1:ITERATION)
    
    A = zeros(6, 6);
    b = zeros(6, 1);
    
    chi = 0;
    last_chi = 1e12;
    
    for index = 1:1:length(points(1, :))
        proj = (new_camera_transform) \ [points(1:3, index);1];
        
        H = twist2PixelDelta(proj(1:3, 1));
        
        e = features(1:2, index) - [proj(1)/proj(3); proj(2)/proj(3)];
        
        chi = chi + norm(e);
        
        huber = huberLossFn(norm(e));
        
        A = A + H'*R_inv*H*huber;
        b = b + H'*R_inv*e*huber;
        
    end
    
    chi = chi / length(points(1, :));
    
    if (chi > last_chi)
        disp('Breaking because chi increased')
        break;
    end
    
    A_full = eye(length(posterior_cov));
    A_full(1:6, 1:6) = A;
    
    b_full = zeros(length(posterior_cov), 1);
    b_full(1:6, 1) = b;
    
    T = inv(inv(posterior_cov) + A_full);
    
    dx = T * (b_full);
    %dx = A\b;
    
    new_camera_transform = new_camera_transform * se3Exp(dx(1:6, 1));
    
    posterior_state = posterior_state + dx;
    
    % transform to the tangent space
    %[posterior_cov, posterior_state] = transform_to_tangent_space(posterior_cov, posterior_state);
    
    
    %STRUCTURE ITERATION
    for index = 1:(length(points(1, :)))
        r = points(1:3, index);
        rcov = points_covs(1:3, (3*index - 2):(3*index));
        
        %proj_r = 
        
        %H = dr2PixelDelta()
    end
    
end


%uncertainty update

posterior_cov = (eye(15) - T*A_full) * posterior_cov * (eye(15) - T*A_full)' + T*A_full*T';

% transform to the tangent space
%[posterior_cov, posterior_state] = transform_to_tangent_space(posterior_cov, posterior_state);
posterior_state
posterior_state(1:6, 1) = zeros(6, 1);

end

