function [cov_new, state_new] = transform_to_tangent_space(cov, state)

x = -state(1:6, 1);
y = state(1:6, 1);
state_new = state;
state_new(1:6, 1) = zeros(6, 1);

Ax = lieBracketMatrixSe3(x);
Ay = lieBracketMatrixSe3(y);

J = eye(6) + 0.5 * Ax + 1/12 * Ax*Ax;

%form the generators for the higher order terms
% these are used like a 3D matrix
Gvx = lieBracketMatrixSe3([1;0;0;0;0;0]);
Gvy = lieBracketMatrixSe3([0;1;0;0;0;0]);
Gvz = lieBracketMatrixSe3([0;0;1;0;0;0]);
Gwx = lieBracketMatrixSe3([0;0;0;1;0;0]);
Gwy = lieBracketMatrixSe3([0;0;0;0;1;0]);
Gwz = lieBracketMatrixSe3([0;0;0;0;0;1]);


%Ay6 = [Ay;Ay;Ay;Ay;Ay;Ay];
%G6 = [Gvx, Gvy, Gvz, Gwx, Gwy, Gwz]';

% now d/dvx (1/12 * Ay * Ay * x) => 1/12 * (Gvx*Ay + Ay*Gvx) * x
T = (1/12) * [(Gvx*Ay + Ay*Gvx)*x, (Gvy*Ay + Ay*Gvy)*x, (Gvz*Ay + Ay*Gvz)*x, (Gwx*Ay + Ay*Gwx)*x, (Gwy*Ay + Ay*Gwy)*x, (Gwz*Ay + Ay*Gwz)*x];
%T = (1/12) * (G6.*Ay6 + Ay6.*G6) * x

% d/dv -1/24 * Ay * Ax * Ax * y => -1/24 * (Gvx*Ay*Ay*y + Ay*Ax*Ax)
R = (-1/24) * ([(Gvx*Ay*Ay*y), (Gvy*Ay*Ay*y), (Gvz*Ay*Ay*y), (Gwx*Ay*Ay*y), (Gwy*Ay*Ay*y), (Gwz*Ay*Ay*y)] + Ay*Ax*Ax);

J = J + T;
J = J + R;

A = eye(15);
A(1:6, 1:6) = J;

cov_new = A*cov*A';

end

