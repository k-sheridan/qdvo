classdef Extrinsics < handle
    %EXTRINSICS this class contains the imu2camera transformation(s) and
    %the scale parameter. It can also be extended to contain any fixed in
    %time quantity.
    
    properties
        imuToCameraTransformContainer = {};
        scaleParameter = 1.0;
    end
    
    methods
        
    end
end

