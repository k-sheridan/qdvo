classdef InertialConstraint
    %INERTIALCONSTRAINT stores the information necessary to form both a
    %preintegrated and full inertial constraint series
    
    properties
        parentKeyframeID = -1;
        childKeyframeID = -1;
        
        imuMeasurementArray = {}; % cell array of IMUMeasurement
    end
    
    methods
        
    end
end

