classdef QuasiDirectErrorTerm_obsFrame
    % error term which seeks to minimize the negative log likelihood of the
    % correspondence distribution by optimizing the imustate of the observing
    % frame and the inverse depth of the landmark in its parent frame.
    % jacobians describe how the error changes with a small perturbation of
    % the state.
    
    properties (Access = private)
        
        % The actual observation data. Used to compute GMM.
        % The ids are stored in the landmarkObservation struct
        landmarkObservation;
        
        % the information matrix for this error term
        Pinv
    end
    
    methods
        function obj = QuasiDirectErrorTerm_obsFrame(landmarkObservation)
            obj.landmarkObservation = landmarkObservation;
            obj.Pinv = inv(obj.landmarkObservation.computeGMMCovariance());
        end
        
        % This function is called by the optimizer. It computes the
        % weighted error and jacobians for both the landmark dinv and frame
        % imustate.
        function [residual, information, jacobians] = computeResidual(obj, graph)
            % get the current data in the graph to compute the residual
            landmarkFrameIdx = graph.getFrameIndex(obj.landmarkObservation.landmarkParentFrameID);
            observationFrameIdx = graph.getFrameIndex(obj.landmarkObservation.observationFrameID);
            landmarkIdx = graph.FrameContainer{landmarkFrameIdx}.getLandmarkIndex(obj.landmarkObservation.landmarkID);
            
            A = graph.FrameContainer{observationFrameIdx}.imustate.R;
            B = graph.FrameContainer{landmarkFrameIdx}.imustate.R;
            T = graph.extrinsics.getImu2CameraTransform(graph.FrameContainer{observationFrameIdx}.camID);
            C = T(1:3, 1:3);
            
            d = graph.FrameContainer{observationFrameIdx}.imustate.p;
            e = graph.FrameContainer{landmarkFrameIdx}.imustate.p;
            f = T(1:3, 4);
            
            dinv = graph.FrameContainer{landmarkFrameIdx}.landmarks{landmarkIdx}.dinv;
            u0 = [graph.FrameContainer{landmarkFrameIdx}.landmarks{landmarkIdx}.bearing; 1];
            
            %transform = inv([A, d; zeros(1, 3), 1] * T) * [B, e; zeros(1, 3), 1] * T;
            
            % compute the residual
            p_obs = C'*A'*B*C*u0*(1/dinv) + C'*A'*(B*f + e - A*f - d);
            %trans = inv([A, d; zeros(1, 3), 1] * T) * [B, e; zeros(1, 3), 1] * T;
            %p_obs = trans(1:3, 1:3) * u0/dinv + trans(1:3, 4);
            
            % project the point into pixel space.
            % NOTE: projJac
            [px, projJac] = graph.FrameContainer{observationFrameIdx}.cameraModel.project(p_obs);
            
            % compute the residual weights.
            weights = obj.landmarkObservation.computeGaussianWeightsRobustly(px, obj.landmarkObservation.theta);
            
            % compute the resultant error
            residual = [0;0]; % In pixel coordinates.
            for idx = (1:length(obj.landmarkObservation.potentialCorrespondenceSet))
                residual = residual + weights(idx) * (px - obj.landmarkObservation.potentialCorrespondenceSet{idx}.pixel);
            end
            
            if nargout >= 2
                information = obj.Pinv;
            end
            
            % Jacobian Computation
            if nargout >= 3
                jacobians = JacobianContainer();
                dPi = [1/p_obs(3), 0, -p_obs(1)/p_obs(3)^2;
                        0, 1/p_obs(3), -p_obs(2)/p_obs(3)^2];
                
                % Observation Jacobian - imustate order: [dp, dphi, dv, dba, dbg]
                J_dt_o = -projJac * dPi * (C'*A');
                J_dphi_o = projJac * dPi * (C' * so3Hat(A'*(B*C*u0*(1/dinv) + B*f + e - d)));
                jacobians.imustateJacobians{1} = {obj.landmarkObservation.observationFrameID, [J_dt_o, J_dphi_o, zeros(2, 9)]};
            end
            
        end
        
    end
end

