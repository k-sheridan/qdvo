syms vx vy vz wx wy wz u_curr rx ry rz u_curr v_curr z_inv_curr

assume([vx;vy;vz;wx;wy;wz], 'real')

rx = u_curr/z_inv_curr
ry = v_curr/z_inv_curr
rz = 1.0/z_inv_curr

% the hat of the se3 lie algebra
A = [0, -wz, wy, vx;
    wz, 0, -wx, vy;
    -wy, wx, 0, vz;
    0, 0, 0, 0]

%compute second order approximation of transform
T = eye(4, 4) + A + A*A/2

T_inv = [T(1:3, 1:3)', -T(1:3, 1:3)*T(1:3, 4); T(4, 1:4)]

v = [vx;vy;vz]
w = [wx;wy;wz]

%compute jacobian for a small camera movement

r = [rx;ry;rz;1] % point position in the current frame (homogenous)

r_p = T_inv*r;

% project onto image plane
u = r_p(1) / r_p(3);
v = r_p(2) / r_p(3);

fprintf('du/dvx: %s \n', subs(diff(u, vx), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))

fprintf('du/dvy: %s\n', subs(diff(u, vy), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))

fprintf('du/dvz: %s\n', subs(diff(u, vz), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))

fprintf('du/dwx: %s\n', subs(diff(u, wx), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))

fprintf('du/dwy: %s\n', subs(diff(u, wy), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))

fprintf('du/dwz: %s\n', subs(diff(u, wz), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))

% v part of jacobian

fprintf('\ndv/dvx: %s \n', subs(diff(v, vx), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))

fprintf('dv/dvy: %s\n', subs(diff(v, vy), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))

fprintf('dv/dvz: %s\n', subs(diff(v, vz), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))

fprintf('dv/dwx: %s\n', subs(diff(v, wx), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))

fprintf('dv/dwy: %s\n', subs(diff(v, wy), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))

fprintf('dv/dwz: %s\n', subs(diff(v, wz), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))


J = collect(jacobian([u;v], [vx,vy,vz,wx,wy,wz]), [u_curr v_curr z_inv_curr])

J(1, 1)


