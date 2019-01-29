classdef InertialErrorTerm < handle
    %INERTIALCONSTRAINT stores the information necessary to form both a
    %preintegrated and full inertial constraint series
    
    properties
        parentFrameID = -1;
        childFrameID = -1;
        
        imuMeasurementArray = {}; % cell array of IMUMeasurement
        
        initialized = false; % has the preintegrated measurement been computed?
        
        % These are the preintegrated measurements with information.
        deltaRotation; % relative rotation between parent and child \in SO(3) (rotation matrix)
        deltaVelocity; % change in velocity from parent to child
        deltaPosition; % change in position from parent to child
        
        % biases do not change!
        
        % order of variables: [drot (so(3)), dvel, dpos, dbias_g, dbias_a]
        biasJacobian; % jacobian used to make the preintegrated deltas a linear function of the biases. These must be recomputed if the bias delta is too large (TBD).
        
        % information matrix
        Pinv; % 15X15 inverse covariance matrix representing the information these deltas add.
        
        
    end
    
    methods
        
    end
end

