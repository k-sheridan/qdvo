classdef Extrinsics < handle
    %EXTRINSICS this class contains the imu2camera transformation(s) and
    %the scale parameter. It can also be extended to contain any fixed in
    %time quantity.
    
    % imu2Camera: transforms a a point in the camera frame to the imu frame
    
    % imu2camera update form: [dp, dphi]
    % gravityVector update form: [dphi]
    
    properties (Access = private)
        imuToCameraTransformContainer = {}; % {{cameraID=1, R1, t1}, {cameraID=2, R2, t2}}
        scaleParameter = 1.0;
        gravityVector = [0;0;-9.8]; % update: g*exp(dphi) ~= g*I + g*hat(dphi) = g*I - hat(g)*dphi
    end
    
    methods
        function [] = addIMU2CameraExtrinsic(obj, camID, R, t)
            %obj.imuToCameraTransformContainer
            obj.imuToCameraTransformContainer{camID} = {camID, R, t};
        end
        
        function [T] = getImu2CameraTransform(obj, camID)
            T = [obj.imuToCameraTransformContainer{camID}{2}, obj.imuToCameraTransformContainer{camID}{3};
                zeros(1, 3), 1];
        end
        
        function [scale] = getScaleParameter(obj)
            scale = obj.scaleParameter;
        end
        
        function [] = updateScaleParameter(obj, dx)
            if length(dx) ~= 1
                error('scale dimension not 1');
            end
            
            obj.scaleParameter = obj.scaleParameter + dx;
        end
    end
end

