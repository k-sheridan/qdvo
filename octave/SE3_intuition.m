% check that exp(se(3) + se(3) + se(3)) == exp(se(3))*exp(se(3))*exp(se(3))

se3_1 = [0;0;0;0.1;0;0.1]
se3_2 = [0.1;-0.1;0.2;0.1;0.1;0]
se3_3 = [0;0.3;-0.1;-0.1;0.2;0]


se3_combine = se3_1+se3_2+se3_3

%not equal
T_mult = se3Exp(se3_1)*se3Exp(se3_2)*se3Exp(se3_2)

T_add = se3Exp(se3_combine)