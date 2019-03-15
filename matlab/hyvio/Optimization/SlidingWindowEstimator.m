classdef SlidingWindowEstimator < handle
    %SLIDINGWINDOWESTIMATOR this is an implementation of a sliding window
    %estimator. It handles marginalization, and optimization of a local
    %window of the graph.
    
    properties
        windowSize; % integer list how many frames into the past are optimized.
        optimizer; % the core of this estimator.
        frameIdsToOptimize = [];
        
    end
    
    methods
        function obj = SlidingWindowEstimator(windowSize)
            obj.windowSize = windowSize;
            obj.optimizer = Optimizer();
        end
        
        % adds the visual error terms.
        function initializeVisualErrorTerms(obj, graph)
            % get the frame ids to be optimized
            obj.frameIdsToOptimize = [];
            for idx = (length(graph.FrameContainer):-1:length(graph.FrameContainer)-obj.windowSize+1)
                obj.frameIdsToOptimize = [obj.frameIdsToOptimize, graph.FrameContainer{idx}.ID];
                
                % while in this loop add the visual constraints for these
                % frames
                if graph.FrameObservationContainer{idx}.frameID ~= graph.FrameContainer{idx}.ID
                    error('Frame Observation Container not associated to the correct frame.')
                end
                
                for vc = graph.FrameObservationContainer{idx}.landmarkObservations
                    % construct and add the visual error term.
                    if vc{1}.observationFrameID ~= graph.FrameContainer{idx}.ID
                        error('landmark observation not associated to the correct frame');
                    end
                    
                    et = QuasiDirectErrorTerm_obsFrame(vc{1});
                    obj.optimizer.addErrorTerm(et);
                end
            end
        end
        
        % initialize the estimator with all error terms involving the
        % latest n states and any inertial constraints between them.
        function [] = initializeGyroOnly(obj, graph)
            % empty the error term container.
            obj.optimizer.clearErrorTerms();
            
            obj.initializeVisualErrorTerms(graph);
            
            % add the inertial constraints between only the frames to be
            % optimized. This excludes the inertial constrain between the
            % oldest frme and the one before it.
            % we expect this will add windowSize-1 inertial constraints
            for idx = (length(graph.InertialConstraintContainer):-1:length(graph.InertialConstraintContainer)-obj.windowSize+2)
                if ~any(graph.InertialConstraintContainer{idx}.parentFrameID == obj.frameIdsToOptimize) ||...
                        ~any(graph.InertialConstraintContainer{idx}.childFrameID == obj.frameIdsToOptimize)
                    error('inertial error term is out of the window!');
                end
                
                et = InertialErrorTerm_gyroOnly(graph.InertialConstraintContainer{idx});
                obj.optimizer.addErrorTerm(et);
            end
        end
        
        % optimize the graph
        function [graph] = optimize(obj, graph)
            % run the optimizer
            graph = obj.optimizer.optimize(graph);
            
            %TODO determine if the optimization failed and revert it.
        end
        
    end
end

