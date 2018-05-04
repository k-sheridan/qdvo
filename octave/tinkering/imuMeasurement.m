
theta = [0.1;0;pi/2]; %ypr

phi = [-pi/2;0;0];

gravity = [0;0;9.8];


eul2rotm(theta')'*gravity
