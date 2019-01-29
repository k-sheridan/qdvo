classdef Extrinsics < handle
    %EXTRINSICS this class contains the imu2camera transformation(s) and
    %the scale parameter. It can also be extended to contain any fixed in
    %time quantity.
    
    properties
        imuToCameraTransformContainer = {}; % {{cameraID=1, T1}, {cameraID=2, T2}}
        scaleParameter = 1.0;
    end
    
    methods
        
    end
end

