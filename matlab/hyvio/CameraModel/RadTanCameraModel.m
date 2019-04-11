classdef RadTanCameraModel
    %Simple Radial Tangential distortion model.
    
    properties
        radiusLookupTable; % nx2 array of [theta, radius], lookup is O(log(n)) with binary search
        coeffs;
        f;
        c;
        fov;
        attenuation; % an array the size of the image which describes the lense attenuation. (default is ones)
        size;
    end
    
    methods
        function obj = RadTanCameraModel(distortionCoeffs, fov, focal, principal, size, n, attenuation)
            %EQUIDISTANTCAMERAMODEL Creates an instance of the camera
            %model. n is optional and determines the resolution of the
            %lookup table.
            % order follows opencv order: [k1, k2, p1, p2]
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
            
            if obj.fov >= pi*0.98
                error('fov too large for rad tan');
            end
            
            
            obj.radiusLookupTable = zeros(n, 2);
            
            obj.radiusLookupTable(:, 1) = linspace(0, tan(obj.fov/2), n);
            
            assert(length(distortionCoeffs) == 4);
            
            obj.coeffs = [distortionCoeffs]; % add a 1 on the lowest order.
            % generate the radius part of the table
            for tableIndex = (1:n)
                multiplier = obj.radiusLookupTable(tableIndex, 1)*(1 + obj.coeffs(1)*obj.radiusLookupTable(tableIndex, 1)^2 +  obj.coeffs(2)*obj.radiusLookupTable(tableIndex, 1)^4);
                
                obj.radiusLookupTable(tableIndex, 2) = multiplier;
            end
        end
        
        function [pixel, projectJacobian] = project(obj, pointInCameraFrame)
            %Project a point in the camera frame into level 0 distorted pixel
            %coordinates. done in the same fashion as kalibr.
            % The projection jacobian maps UNIT PLANE bearing error to
            % pixel error.
            
            
            if (pointInCameraFrame(3) <= 1e-8)
                error('point is behind camera');
            end
            
            pointInCameraFrame = pointInCameraFrame / pointInCameraFrame(3); % make point homogenous.
            
            r = norm(pointInCameraFrame(1:2, 1));
            
            uvd = pointInCameraFrame(1:2)*(1 + obj.coeffs(1)*r^2 +  obj.coeffs(2)*r^4);
            uvd = uvd + [2*obj.coeffs(3)*pointInCameraFrame(1)*pointInCameraFrame(2) + obj.coeffs(4)*(r^2 + 2*pointInCameraFrame(1)^2); 
                2*obj.coeffs(4)*pointInCameraFrame(1)*pointInCameraFrame(2) + obj.coeffs(3)*(r^2 + 2*pointInCameraFrame(2)^2)];
            
            
            pixel = [obj.f(1)*uvd(1) + obj.c(1); obj.f(2)*uvd(2) + obj.c(2)];
            
            if ~obj.isPixelOnImage(pixel)
                error('pixel not on image');
            end
            
            % if the projection jacobian is desired compute it.
            if (nargout == 2)
                % [dx; dy] = J * [du; dv] <= (unit plane)
                delta = 1e-4;
                bearing = [pointInCameraFrame(1:2); 1];
                
                u2 = obj.project(bearing + [delta;0;0]);
                u1 = obj.project(bearing + [-delta;0;0]);
                
                v2 = obj.project(bearing + [0;delta;0]);
                v1 = obj.project(bearing + [0;-delta;0]);
                
                projectJacobian = [1/(2*delta)*(u2-u1), 1/(2*delta)*(v2-v1)];
            end
        end
        
        function [bearing, unprojectJacobian] = unproject(obj, pixel)
            % undistorts the pixel onto the image plane in normalized
            % (metric) coordinates.
            % 
            
            
            uvd = [(pixel(1) - obj.c(1)) / obj.f(1); (pixel(2) - obj.c(2)) / obj.f(2)];
            
            psi = atan2(uvd(2), uvd(1)); % used to reproduce the unit vector robustly
            
            
            % find the corrected radius
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
            
            newRadius = obj.radiusLookupTable(mid, 1);
            
            bearing0 = newRadius * [cos(psi); sin(psi)];
            
            % finally, iteratively refine the bearing.
            bearing = bearing0;
            for iter = (1:10)
                % standard nonlinear least squares
                [pxCurr, projJac] = obj.project([bearing; 1]);
                r = pxCurr - pixel;
                delta = (projJac'*projJac)\(-projJac' * r);
                bearing = bearing + delta;
            end
            
            
            % if the unproject jacobian is desired use the new bearing to
            % compute the projection jacobian and invert it.
            if (nargout == 2)
                [px, pJ] = obj.project(bearing);
                
                unprojectJacobian = inv(pJ);
            end
            
        end
        
        
        function [onImage] = isPixelOnImage(obj, px)
            %persistent buffer;
            %persistent attenuationMinimum; 
            attenuationMinimum = 0;
            buffer = 2;
            if px(1) <= buffer || px(1) >= (obj.size(1) - buffer)
                onImage = false;
                return
            end
            
            if px(2) <= buffer || px(2) >= (obj.size(2) - buffer)
                onImage = false;
                return
            end
            
%             if obj.attenuation(round(px(2)), round(px(1))) <= attenuationMinimum
%                 onImage = false;
%                 return
%             end
            
            onImage = true;
        end
    end
end
