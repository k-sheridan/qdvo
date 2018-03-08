function [new_vector] = rodrigues(angle, vector)
%rotate a vector by an angle axis

theta = norm(angle);

if(theta < 1e-8)
    new_vector = rodriguesSmall(angle, vector);
else

    k = angle/theta;

    new_vector = vector*cos(theta) + cross(k, vector)*sin(theta) + k*dot(k, vector)*(1-cos(theta));
end

end

