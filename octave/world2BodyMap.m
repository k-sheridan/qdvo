function [J] = world2BodyMap(quat)
% forms a transformation map to transform uncertainty into the body frame 
% order: tx ty tz qw qx qy qz

    % apply the opposite rotation 

    r0 = quat(1);
    r1 = -quat(2);
    r2 = -quat(3);
    r3 = -quat(4);
    
    q0 = r0;
    q1 = r1;
    q2 = r2;
    q3 = r3;

    A = [r0, -r1, -r2, -r3;
        r1, r0, r3, -r2;
        r2, -r3, r0, r1;
        r3, r2, -r1, r0];
    
    R = [(1-2*q2^2-2*q3^2), 2*(q1*q2 + q0*q3), 2*(q1*q3-q0*q2);
        2*(q1*q2-q0*q3), (1-2*q1^2-2*q3^2), 2*(q2*q3+q0*q1);
        2*(q1*q3+q0*q2), 2*(q2*q3-q0*q1), (1-2*q1^2-2*q2^2)];
    
    J = [R, zeros(3, 4);
        zeros(4, 3), A];

end

