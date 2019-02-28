classdef inertialErrorTerm_gyroOnly
    %INERTIALERRORTERM_GYROONLY constrains two consecutive frames by
    %rotation only. Used to optimize the imustate of both frames. Only
    %provides information about the gyro biases and relative rotations.
    
    properties
        preintegratedIMUMeasurement;
    end
    
    methods
        function obj = inertialErrorTerm_gyroOnly(preintegratedIMUMeasurement)
            obj.preintegratedIMUMeasurement = preintegratedIMUMeasurement;
            assert(obj.preintegratedIMUMeasurement.initialized);
        end
        
        % This residual is 3 dimensional and is a log map.
        % r = so3Log(dR * R_i'*R_j)
        function [residual, jacobians] = computeResidual(obj, graph)
            % get the current gyro biases and rotations form the parent and
            % child frame.
            childFrameIdx = graph.getFrameIndex(obj.preintegratedIMUMeasurement.childFrameID);
            parentFrameIdx = graph.getFrameIndex(obj.preintegratedIMUMeasurement.parentFrameID);
            
            Ri = graph.FrameContainer{parentFrameIdx}.imustate.R;
            Rj = graph.FrameContainer{childFrameIdx}.imustate.R;
            
            residual = so3Log(obj.preintegratedIMUMeasurement.deltaR' * Ri' * Rj);
            
            % Compute residuals
            if nargout == 2
                
            end
                
        end
    end
end

