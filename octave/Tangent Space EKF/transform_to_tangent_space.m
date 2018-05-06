function [cov_new] = transform_to_tangent_space(cov, twist)

x = -twist;
y = twist;
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

% now d/dvx (1/12 * Ay * Ay * x) = 1/12 * (Gvx*Ay + Ay*Gvx) * x
T = (1/12) * [(Gvx*Ay + Ay*Gvx)*x, (Gvy*Ay + Ay*Gvy)*x, (Gvz*Ay + Ay*Gvz)*x, (Gwx*Ay + Ay*Gwx)*x, (Gwy*Ay + Ay*Gwy)*x, (Gwz*Ay + Ay*Gwz)*x];

J = J + T

end

