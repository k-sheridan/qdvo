classdef VisionGyroBA < handle
    %VISIONGYROBA estimates the structure, camera poses, and gyro bias over
    %the whole graph.
    
    properties
        optimizer = Optimizer();
    end
    
    methods
        % constructs error terms, and optimizes the graph.
        function [graph] = optimize(obj, graph)
            % clear the previous error terms from the optimizer
            obj.optimizer.clearErrorTerms();
            % add inertial (gyro) constraints
            for ic = graph.InertialConstraintContainer
                E = InertialErrorTerm_gyroOnly(ic{1});
                obj.optimizer.addErrorTerm(E);
            end
            
            % add visual constraints
            for idx = (1:length(graph.FrameObservationContainer))
                for vc = graph.FrameObservationContainer{idx}.landmarkObservations
                    E = QuasiDirectErrorTerm_obsFrame_landmarkDinv(vc{1});
                    obj.optimizer.addErrorTerm(E);
                end
            end
            
            obj.optimizer.numberOfErrorTerms()
            
            % run the optimizer
            graph = obj.optimizer.optimize(graph);
        end
    end
end

