classdef VisualBA < handle
    %VISIONGYROBA estimates the structure, camera poses, and gyro bias over
    %the whole graph.
    
    properties
        optimizer = Optimizer();
    end
    
    methods
        
        function [] = initialize(obj, graph)
            % clear the previous error terms from the optimizer
            obj.optimizer.clearErrorTerms();
            
            % add visual constraints
            for idx = (1:length(graph.FrameObservationContainer))
                if graph.FrameContainer{idx}.isKeyframe || idx == length(graph.FrameContainer)
                    for vc = graph.FrameObservationContainer{idx}.landmarkObservations
                        E = QuasiDirectErrorTerm_obsFrame_landmarkDinv(vc{1});
                        obj.optimizer.addErrorTerm(E);
                    end
                end
            end
            
            obj.optimizer.numberOfErrorTerms()
        end
        
        % optimizes the graph.
        function [graph] = optimize(obj, graph)
            
            % run the optimizer
            graph = obj.optimizer.optimize(graph);
        end
    end
end

