syms qw qx qy qz vx vy vz wx wy wz rx ry rz u v zinv

assume([wx, wy, wz, vx, vy, vz], 'real')

% twist to delta quaternion
B = 0.5*[0 wz -wy wx;
        -wz 0 wx wy;
        wy -wx 0 wz;
        -wx -wy -wz 0]
    
jacobian(B*[qx;qy;qz;qw], [wx;wy;wz])


%rx = u/zinv;
%ry = v/zinv;
%rz = 1.0/zinv;

%compute jacobian for a small camera movement

r = [rx;ry;rz;1]; % point position in the current frame (homogenous)

% the hat of the se3 lie algebra
A = [0, -wz, wy, vx;
    wz, 0, -wx, vy;
    -wy, wx, 0, vz;
    0, 0, 0, 0]

%compute first order approximation of transform
T = eye(4, 4) + A

T_inv = [T(1:3, 1:3)', -T(1:3, 1:3)*T(1:3, 4); T(4, 1:4)]

v = [vx;vy;vz]
w = [wx;wy;wz]

r_p = T_inv*r; % project into next camera

% project onto image plane
u = r_p(1) / r_p(3);
v = r_p(2) / r_p(3);

% approximate mapping from twist into pixel bearing around the unit twist
J = subs(jacobian([u;v], [vx;vy;vz;wx;wy;wz]), [vx;vy;vz;wx;wy;wz], [0;0;0;0;0;0])