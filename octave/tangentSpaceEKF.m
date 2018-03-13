
% first 6 dimensions are the tangent space around the current pose (stored externally)
T = eye(4);
%state = [tx, ty, tz, thetax, thetay, thetaz, bdx, bdy, bdz, bwx, bwy, bwz, bax, bay, baz]
state = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; pi/4; 0; 0; 0];

Sigma = zeros(15);
Sigma(1:6, 1:6) = eye(6)*0.001;
Sigma(7:15, 7:15) = eye(9)*0.001;


dt = 0.01;

for t = (0:dt:10)
    [state, Sigma, T] = process(state, Sigma, T, dt);
    drawState(T, Sigma);
end

