classdef correspondenceErrorTerm_obsFrame_landmarkDinv
    % error term which seeks to minimize the negative log likelihood of the
    % correspondence distribution by optimizing the imustate of the observing
    % frame and the inverse depth of the landmark in its parent frame.
    
    properties (Access = private)
        
        % The actual observation data. Used to compute GMM.
        % The ids are stored in the landmarkObservation struct
        landmarkObservation;
        
        % the information matrix for this error term
        Pinv
    end
    
    methods
        function obj = correspondenceErrorTerm_obsFrame_landmarkDinv(landmarkObservation)
            obj.landmarkObservation = landmarkObservation;
            obj.Pinv = inv(obj.landmarkObservation.computeGMMCovariance());
        end
        
        % This function is called by the optimizer. It computes the
        % weighted error and jacobians for both the landmark dinv and frame
        % imustate.
        function [residual, jacobians] = computeResidual(obj, graph)
            % get the current data in the graph to compute the residual
            landmarkFrameIdx = graph.getFrameIndex(obj.landmarkObservation.landmarkFrameID);
            observationFrameIdx = graph.getFrameIndex(obj.landmarkObservation.observationFrameID);
            landmarkIdx = graph.FrameContainer{landmarkFrameIdx}.getLandmarkIndex(obj.landmarkObservation.landmarkID);
            
            T_w_simu = graph.FrameContainer{landmarkFrameIdx}.imustate.poseTransform();
            T_w_oimu = graph.FrameContainer{observationFrameIdx}.imustate.poseTransform();
            
            T_i_c = graph.extrinsics.getImu2CameraTransform(graph.FrameContainer{observationFrameIdx}.camID);
            
            dinv = graph.FrameContainer{observationFrameIdx}.landmarks{landmarkIdx}.dinv;
            u0 = [graph.FrameContainer{observationFrameIdx}.landmarks{landmarkIdx}.bearing; 1];
            
            % [p_obs;1] = inv(T_w_oimu * T_i_c) * (T_w_simu * T_i_c) * [u0 *
            % (dinv)^-1; 1]
            
            
        end
        
    end
end

