 tangent = [0.01;0.02;0.015;0.01; 0.005; 0.015];
 
 delta = 1e-3;
 
 J = zeros(6);
 
 for col = (1:6)
     
     delta_vec = zeros(6, 1);
     delta_vec(col) = delta;
     
     J(:, col) = (1/(2*delta)) * (se3Log(se3Exp(-tangent) * se3Exp(tangent + delta_vec)) - se3Log(se3Exp(-tangent) * se3Exp(tangent - delta_vec)));
     
 end
 
 J