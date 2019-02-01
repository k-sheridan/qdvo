classdef Extrinsics < handle
    %EXTRINSICS this class contains the imu2camera transformation(s) and
    %the scale parameter. It can also be extended to contain any fixed in
    %time quantity.
    
    % imu2Camera: transforms a a point in the camera frame to the imu frame
    
    properties
        imuToCameraTransformContainer = {}; % {{cameraID=1, R1, t1}, {cameraID=2, R2, t2}}
        scaleParameter = 1.0;
    end
    
    methods
        function [] = addIMU2CameraExtrinsic(obj, camID, R, t)
            obj.imuToCameraTransformContainer{end+1} = {camID, R, t};
        end
    end
end

