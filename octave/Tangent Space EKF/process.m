function [state, Sigma, pose_transform] = process(state, Sigma, pose_transform, dt)
    new_state = convolveState(state, dt);
    T = se3Exp(new_state(1:6, 1)); % compute the transform that the tangent vector has undergone
    
    pose_transform = pose_transform * T;
    
    % numerically computing the Map into the next state space
    delta = 1e-3;
    J = zeros(length(state));
    for index = (1:length(state))
        upper = state;
        lower = state;
        
        upper(index) = upper(index) + delta;
        lower(index) = lower(index) - delta;
        
        upper_out = convolveState(upper, dt);
        lower_out = convolveState(lower, dt);
        
        J(:, index) = (upper_out - lower_out) / (2*delta);
    end
    
    
    
    % propagate the uncertainty, then transform the the tangent space of
    % the new pose
    Q = processNoise(dt);
    
    state = new_state;
    [Sigma, state] = transform_to_tangent_space((J*Sigma*J' + Q), new_state)
end

function [x] = convolveState(x0, dt)
    
    dr =  x0(7:9, 1)*dt + 0.5*dt^2*x0(13:15, 1);
    
    w = x0(10:12, 1)*dt;
    
    %compute the new tangent from the rotation and translation
    x(1:6, 1) = se3Log(se3Exp(x0(1:6, 1)) * se3Exp([dr;w]));
    
    %R = so3Exp(x0(10:12, 1)*dt);
    
    x(7:9, 1) = (x0(7:9, 1) + x0(13:15, 1)*dt);
    
    x(10:12, 1) = x0(10:12, 1);
    
    x(13:15, 1) = (x0(13:15, 1));
end


function [Q] = processNoise(dt)
    Q = eye(15)* dt*0.01;
    Q(1:3, 1:3) = eye(3)*50*dt;
    Q(7:9, 7:9) = eye(3)*dt*2;
    Q(10:15, 10:15) = eye(6)*dt*5;
end

