function [posterior_cov, posterior_state, post_pose] = gyro_update(prior_tangent_space_uncertainty, prior_state, prior_pose, omega)
%standard kalman update for gyro 

H = zeros(3, 15);
H(1, 10) = 1;
H(2, 11) = 1;
H(3, 12) = 1;

gyro_cov = (pi/3600)^2;

R = diag([gyro_cov, gyro_cov, gyro_cov]);

S = R + H*prior_tangent_space_uncertainty*H';

K = prior_tangent_space_uncertainty*H' / (S);

dx = K*(omega - prior_state(10:12, 1));

posterior_state = prior_state + dx;

posterior_cov = (eye(15, 15) - K*H) * prior_tangent_space_uncertainty * (eye(15, 15) - K*H)' + K * R * K';

post_pose = prior_pose * se3Exp(dx(1:6, 1));

end

