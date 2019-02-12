classdef Extrinsics < handle
    %EXTRINSICS this class contains the imu2camera transformation(s) and
    %the scale parameter. It can also be extended to contain any fixed in
    %time quantity.
    
    % imu2Camera: transforms a a point in the camera frame to the imu frame
    
    properties (Access = private)
        imuToCameraTransformContainer = {}; % {{cameraID=1, R1, t1}, {cameraID=2, R2, t2}}
        scaleParameter = 1.0;
        gravityVector = [0;0;-9.8]; % update: g*exp(dphi) ~= g*I + g*hat(dphi) = g*I - hat(g)*dphi
    end
    
    methods
        function [] = addIMU2CameraExtrinsic(obj, camID, R, t)
            obj.imuToCameraTransformContainer
            %obj.imuToCameraTransformContainer{camID} = {camID, R, t};
        end
        
        function [imu2CameraR, imu2Camerat] = getImu2CameraTransform(obj, camID)
            imu2CameraR = obj.imuToCameraTransformContainer{camID}{2};
            imu2Camerat = obj.imuToCameraTransformContainer{camID}{3};
        end
    end
end

