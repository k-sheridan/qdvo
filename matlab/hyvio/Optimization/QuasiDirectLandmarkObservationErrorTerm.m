classdef QuasiDirectLandmarkObservationErrorTerm
    %QUASIDIRECTERRORTERM computes the residual and jacobians wrt the
    %observation frame pose, landmark inverse depth, landmark parent frame
    %pose.
    
    properties
        lo;
        loCov; % covariance of the gaussian mixture model.
    end
    
    methods
        function obj = QuasiDirectErrorTerm(landmarkObservation)
            % Constructs an error term which can be used in an optimizer. 
            obj.lo = landmarkObservation;
            obj.loCov = obj.lo.computeGMMCov();
        end
        
        function [residual, jacobianContainer] = computeResidual(graph)
            
        end
        
    end
end

