
% first 6 dimensions are the tangent space around the current pose (stored externally)
T = eye(4);
%state = [tx, ty, tz, thetax, thetay, thetaz, bdx, bdy, bdz, bwx, bwy, bwz]
state = [0; 0; 0; 0; 0; 0; 1; 0; 0; 0; 0; pi/4];

