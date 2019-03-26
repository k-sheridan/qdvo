classdef SlidingWindowEstimator < handle
    %SLIDINGWINDOWESTIMATOR this is an implementation of a sliding window
    %estimator. It handles marginalization, and optimization of a local
    %window of the graph.
    
    properties
        windowSize; % integer list how many frames into the past are optimized.
        optimizer; % the core of this estimator.
        
        activeFrameIDs; % the list of frame IDs which are optimized in the SWE
    end
    
    methods
        function obj = SlidingWindowEstimator(windowSize)
            obj.windowSize = windowSize;
            obj.optimizer = Optimizer();
        end
        
        % adds the visual error terms.
        function initializeVisualErrorTerms(obj, graph)
            
            obj.activeFrameIDs = [];
            
            % get the frame ids to be optimized
            for idx = (1:length(graph.FrameContainer))
                
                if graph.FrameContainer{idx}.status ~= FrameStatus.ACTIVE
                    continue;
                end
                
                if ~graph.FrameContainer{idx}.isKeyframe && graph.FrameContainer{idx}.status == FrameStatus.ACTIVE
                    error('non keyframe is marked active!')
                end
                
                obj.activeFrameIDs = [obj.activeFrameIDs, graph.FrameContainer{idx}.ID];
                
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
                    
                    lidx = graph.getFrameIndex(vc{1}.landmarkParentFrameID);
                    
                    if graph.FrameContainer{lidx}.landmarks{graph.FrameContainer{lidx}.getLandmarkIndex(vc{1}.landmarkID)}.status...
                            ~= LandmarkStatus.ACTIVE
                        error('correspondence refers to an inactive landmark!');
                    end
                    
                    
                    % construct and add the error term.
                    et = QuasiDirectErrorTerm_obsFrame_landmarkDinv_sourceFrame(vc{1});
                    obj.optimizer.addErrorTerm(et);
                end
            end
            
            obj.activeFrameIDs
            
            if length(obj.activeFrameIDs) > obj.windowSize
                fprintf('Too many active frames in the window: %i \n', length(obj.activeFrameIDs));
            end
            
        end
        
        % initialize the estimator with all error terms involving the
        % latest n states (visual constraints only).
        function [] = initializeVisionOnly(obj, graph)
            % empty the error term container.
            obj.optimizer.clearErrorTerms();
            
            obj.initializeVisualErrorTerms(graph);
            
        end
        
        
        
        % optimize the graph
        function [graph] = optimize(obj, graph)
            % run the optimizer
            graph = obj.optimizer.optimize(graph);
            
            %TODO determine if the optimization failed and revert it.
        end
        
        % removes the oldest frame & its landmarks from the window and approximates its
        % information with a quadratic error (prior)
        function [graph] = marginalizeFrame(obj, graph, frameID)
            
            % this will check that the frame id is even in the state.
            indices = obj.optimizer.getPrior().indexHandler.getImustateIndices(frameID);
            
            % marginalize the landmarks in this keyframe.
            landmarkIDArray = {};
            fidx = graph.getFrameIndex(frameID);
            idx = 1;
            for l = graph.FrameContainer{fidx}.landmarks
                if l{1}.status == LandmarkStatus.ACTIVE
                    landmarkIDArray{end+1} = {frameID, l{1}.ID};
                end
                
                graph.FrameContainer{fidx}.landmarks{idx}.status = LandmarkStatus.MARGINALIZED;
                idx=idx+1;
            end
            
            obj.optimizer.marginalizeLandmarkBatch(landmarkIDArray);
            
            % marginalize the keyframe
            obj.optimizer.marginalizeImustate(frameID);
            
            graph.FrameContainer{fidx}.status = FrameStatus.INACTIVE;
        end
        
        
    end
end

