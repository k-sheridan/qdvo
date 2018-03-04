syms vx vy vz wx wy wz u_curr rx ry rz u_curr v_curr z_inv_curr

rx = u_curr/z_inv_curr
ry = v_curr/z_inv_curr
rz = 1.0/z_inv_curr

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

% how does changing the position of the point in the current frame change the
% u and v

fprintf('\ndu/ddepth: %s\n', subs(diff(u, z_inv_curr), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))

fprintf('dv/ddepth: %s\n', subs(diff(v, z_inv_curr), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0]))


J = subs(jacobian([u;v], [vx,vy,vz,wx,wy,wz,z_inv_curr]), [vx,vy,vz,wx,wy,wz], [0, 0, 0, 0, 0, 0])

latex(J)

% test my logic 

z = [0.1; 0.1]

R = [0.00001, 0;
    0, 0.00001]

current_z = [0.099; 0.1001; 0.1]

T = eye(4, 4)

%order vx, vy, vz, wx, wy, wz, z^-1

P = [1, 0, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0, 0;
    0, 0, 0, 0.0001, 0, 0, 0;
    0, 0, 0, 0, 0.0001, 0, 0;
    0, 0, 0, 0, 0, 0.0001, 0;
    0, 0, 0, 0, 0, 0, 1;]

last_norm = z - current_z(1:2);

for it = (1:100)

    H = double(subs(J, [u_curr; v_curr; z_inv_curr], current_z))
    
    err = z - current_z(1:2);
    
    K = inv(inv(P) + H'*inv(R)*H) * H' * inv(R);
    
    delta = K*err
    
    dT = se3Exp(delta(1:6));
    
    T = dT * T;
    
    pos = (1/current_z(3)) * [current_z(1:2); 1];
   
    new_pos = inv(T) * [pos;1];
    
    current_z = [new_pos(1)/new_pos(3); new_pos(2)/new_pos(3); 1/new_pos(3)];
    
    P = inv(inv(P) + H'*inv(R)*H)
    
    if (last_norm < norm(z - current_z(1:2)))
        sprintf('error increased at it %d', it)
        break;
    end
    last_norm = norm(z - current_z(1:2))
end

T
