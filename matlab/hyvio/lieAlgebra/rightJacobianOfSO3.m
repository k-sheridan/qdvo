function [Jr] = rightJacobianOfSO3(phi)
    theta = norm(phi);
    
    if theta < 1e-12
        %warning('right jacobian cannot be computed, phi too small.');
        Jr = eye(3);
    else
        unitHat = so3Hat(phi/theta);
        
        Jr = eye(3) - (1 - cos(theta))/theta * unitHat + (theta - sin(theta))/theta * unitHat^2;
    end
end

