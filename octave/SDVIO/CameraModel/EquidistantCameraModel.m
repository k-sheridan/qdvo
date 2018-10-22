classdef EquidistantCameraModel
    %EQUIDISTANTCAMERAMODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        radiusLookupTable; % nx2 array of [theta, radius], lookup is O(log(n)) with binary search
        order;
        coeffs;
        f;
        c;
        fov;
        attenuation; % an array the size of the image which describes the lense attenuation. (default is ones)
        size;
    end
    
    methods
        function obj = EquidistantCameraModel(distortionCoeffs, fov, focal, principal, size, n, attenuation)
            %EQUIDISTANTCAMERAMODEL Creates an instance of the camera
            %model. n is optional and determines the resolution of the
            %lookup table.
            % This adds a 1 to the front of the distortion coeff vec.
            if (nargin <= 6)
                attenuation = ones(size);
                if (nargin <= 5)
                    n = 10000;
                end
            end
            
            obj.attenuation = attenuation;
            obj.size = size;
            
            obj.f = focal;
            obj.c = principal;
            obj.fov = fov;
            
            thetaMax = fov/2; % the maximum theta value in the lookup table
            obj.radiusLookupTable = zeros(n, 2);
            
            obj.radiusLookupTable(:, 1) = linspace(0, thetaMax, n);
            
            obj.coeffs = [1, distortionCoeffs]; % add a 1 on the lowest order.
            obj.order = (1:2:1 + (length(obj.coeffs)-1) * 2) % generate the orders of each term
            % generate the radius part of the table
            for tableIndex = (1:n)
                radius = 0;
                for coeffIndex = (1:length(obj.coeffs))
                    radius = radius + obj.coeffs(coeffIndex) * obj.radiusLookupTable(tableIndex, 1)^obj.order(coeffIndex);
                end
                
                obj.radiusLookupTable(tableIndex, 2) = radius;
            end
        end
        
        function [pixel] = project(obj, pointInCameraFrame)
            %Project a point in the camera frame into level 0 distorted pixel
            %coordinates. done in the same fashion as kalibr.
            
            if (pointInCameraFrame(3) <= 1e-8)
                warning('point is behind camera')
            end
            
            theta = atan2(sqrt(pointInCameraFrame(1)^2 + pointInCameraFrame(2)^2), abs(pointInCameraFrame(3)));
            psi = atan2(pointInCameraFrame(2), pointInCameraFrame(1));
            
            radius = 0;
            for coeffIndex = (1:length(obj.coeffs))
                radius = radius + obj.coeffs(coeffIndex) * theta^obj.order(coeffIndex);
            end
            
            bearingDistorted = [radius * cos(psi); radius * sin(psi); 1];
            
            pixel = [bearingDistorted(1) * obj.f(1) + obj.c(1); bearingDistorted(2) * obj.f(2) + obj.c(2)];
        end
        
        function [bearing] = unproject(obj, pixel)
            % undistorts the pixel onto the image plane in normalized
            % (metric) coordinates.
            % the only way to break this is by asking for a out of bounds
            % pixel like [1e10; 0]
            
            % step 1 normalize the pixel to distorted homogenous
            % coordinates.
            
            uvd = [(pixel(1) - obj.c(1)) / obj.f(1); (pixel(2) - obj.c(2)) / obj.f(2)];
            
            psi = atan2(uvd(2), uvd(1)); % used to reproduce the unit vector robustly
            
            % step 2 binary search the lookup table for the distorted
            % radius
            radiusDesired = norm(uvd); % distorted radius.
            
            % this assumes that radius increases with theta
            high = length(obj.radiusLookupTable(:, 1));
            low = 1;
            mid = floor((high - low) / 2);
            
            while (1)
                
                eval = obj.radiusLookupTable(mid, 2);
           
                if (high - 1 <= low)
                    break;
                end
                
                
                if (radiusDesired > eval)
                    low = mid;
                    mid = floor(low + (high - low) / 2);
                elseif (radiusDesired < eval)
                    high = mid;
                    mid = floor(low + (high - low) / 2);
                else
                    break;
                end
                
            end
            
            theta0 = obj.radiusLookupTable(mid, 2);
            
            % FINAL STEP minimize the squared radius error
            % r(theta0 + dtheta) ~= r0 + dr_dth(r0)*dtheta
            % rd - r0 - dr_dth(r0)*dtheta = 0
            % => dtheta = (rd - r0) / dr_dth(r0)
            % iterate until convergence
            
            theta = theta0;
            for iter = (1:20) % should converge way before 20 iters depending on table resolution
                der = obj.distortionDerivativeFn(theta);
                
                if (norm(der) <= 1e-8)
                    break;
                end
                
                theta = theta + (radiusDesired - obj.distortionFn(theta)) / der;
            end
            
            bearing = [tan(theta) * [cos(psi); sin(psi)]; 1];
        end
        
        function [radius] = distortionFn(obj, theta)
            % evaluates the equidistant distortion function for this model
            
            radius = 0;
            for coeffIndex = (1:length(obj.coeffs))
                 radius = radius + obj.coeffs(coeffIndex) * theta^obj.order(coeffIndex);
            end
        end
        
        function [dradius_dtheta] = distortionDerivativeFn(obj, theta)
            % evaluates the derivative of the equidistant distortion function for this model
            
            dradius_dtheta = 0;
            for coeffIndex = (1:length(obj.coeffs))
                 dradius_dtheta = dradius_dtheta + obj.order(coeffIndex) * obj.coeffs(coeffIndex) * theta^(obj.order(coeffIndex) - 1);
            end
        end
    end
end

