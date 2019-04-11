classdef FOVCameraModel
    %Simple Radial Tangential distortion model.
    
    properties
        w;
        f;
        c;
        fov;
        attenuation; % an array the size of the image which describes the lense attenuation. (default is ones)
        size;
    end
    
    methods
        function obj = FOVCameraModel(w, fov, focal, principal, size, n, attenuation)
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
            
            
            obj.w = w;
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
            
            r_u = norm(pointInCameraFrame(1:2, 1));
            
            psi = atan2(pointInCameraFrame(2), pointInCameraFrame(1));
            
            uvd = [cos(psi);sin(psi)] * 1/obj.w * atan(2*r_u*tan(obj.w/2));
            
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
            
            
            bearing = tan(norm(uvd) * obj.w) / (2 * tan(obj.w / 2));
            
            
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

