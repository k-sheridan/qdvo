classdef InertialErrorTerm_gyroOnly
    %INERTIALERRORTERM_GYROONLY constrains two consecutive frames by
    %rotation only. Used to optimize the imustate of both frames. Only
    %provides information about the gyro biases and relative rotations.
    
    properties
        preintegratedIMUMeasurement;
    end
    
    methods
        function obj = InertialErrorTerm_gyroOnly(preintegratedIMUMeasurement)
            obj.preintegratedIMUMeasurement = preintegratedIMUMeasurement;
            assert(obj.preintegratedIMUMeasurement.initialized);
        end
        
        % This residual is 3 dimensional and is a log map.
        % r = so3Log(dR * R_i'*R_j)
        function [residual, information, jacobians] = computeResidual(obj, graph)
            % get the current gyro biases and rotations form the parent and
            % child frame.
            childFrameIdx = graph.getFrameIndex(obj.preintegratedIMUMeasurement.childFrameID);
            parentFrameIdx = graph.getFrameIndex(obj.preintegratedIMUMeasurement.parentFrameID);
            
            if all(graph.FrameContainer{parentFrameIdx}.imustate.biases ~= obj.preintegratedIMUMeasurement.biasLinearlizationPoint)
                error('biases of preintegrated imu are not equal to parent biases!');
            end
            
            Ri = graph.FrameContainer{parentFrameIdx}.imustate.R;
            bi = graph.FrameContainer{parentFrameIdx}.imustate.biases;
            Rj = graph.FrameContainer{childFrameIdx}.imustate.R;
            bj = graph.FrameContainer{childFrameIdx}.imustate.biases;
            
            dR = obj.preintegratedIMUMeasurement.deltaR' * Ri' * Rj;
            residual = [so3Log(dR); (bi-bj)];
            
            % set information matrix
            if nargout >= 2
                information = inv([obj.preintegratedIMUMeasurement.P(4:6, 4:6), zeros(3, 6);
                                zeros(6, 3), obj.preintegratedIMUMeasurement.P(10:15, 10:15)]);
            end
            
            % compute jacobians
            if nargout >= 3
                jacobians = JacobianContainer();
                Jr_inv = inv(rightJacobianOfSO3(residual));
                % 1) parent frame jacobians (i).
                drR_dphi_i = -Jr_inv * Rj'*Ri;
                drR_bg = -Jr_inv * dR' * obj.preintegratedIMUMeasurement.dDR_dbg;
                
                J1 = [[zeros(3), drR_dphi_i, zeros(3, 6), drR_bg];
                      [zeros(6, 9), eye(6)]];
                
                jacobians.imustateJacobians{1} = {obj.preintegratedIMUMeasurement.parentFrameID, J1};
                
                % 2) child frame jacobians (j).
                drR_dphi_j = Jr_inv;
                
                J2 = [[zeros(3), drR_dphi_j, zeros(3, 9)];
                      [zeros(6, 9), -eye(6)]];
                
                jacobians.imustateJacobians{2} = {obj.preintegratedIMUMeasurement.childFrameID, J2};
            end
                
        end
    end
end

