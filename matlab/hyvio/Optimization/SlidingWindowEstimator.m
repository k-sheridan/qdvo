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
                        graph.FrameContainer{lidx}.landmarks{graph.FrameContainer{lidx}.getLandmarkIndex(vc{1}.landmarkID)}.status
                        fprintf('correspondence refers to an inactive or marginalized landmark!\n');
                        continue;
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
        
        % This will marginalize a keyframe and its landmarks if necessary
        % according to a marginalization strategy based on number of
        % criteria.
        function [graph] = runMarginalizationStrategy(obj, graph)
            s = Settings();
            
            obj.activeFrameIDs = sort(obj.activeFrameIDs); % ensure the ids are in ascending order
            
            if length(obj.activeFrameIDs) < obj.windowSize
                % we are good, no need to marginalize any keyframes yet.
                return;
            end
            
            % Step 1 see if a keyframe has a low feature percentage.
            minimumFeaturePercentage = 0.04;
            keyframeFeatureCount = zeros(1, length(obj.activeFrameIDs));
            assert(graph.FrameContainer{end}.isKeyframe && graph.FrameContainer{end}.status == FrameStatus.ACTIVE);
            % look at the correspondence models to see what features are
            % being used.
            foidx = graph.getFrameObservationsIndex(graph.FrameContainer{end}.ID);
            for lo = graph.FrameObservationContainer{foidx}.landmarkObservations
                parentID = lo{1}.landmarkParentFrameID;
                idx = find(parentID == obj.activeFrameIDs);
                keyframeFeatureCount(idx) = keyframeFeatureCount(idx) + 1;
            end
            
            featureRatios = keyframeFeatureCount/sum(keyframeFeatureCount)
            
            % marginalize the newest keyframe which has a low Feature
            % count. and is not the two newest keyframes.
            if any(featureRatios(1:end-2) < minimumFeaturePercentage)
                
                temp = find(featureRatios(1:end-2) < minimumFeaturePercentage);
                idx = temp(end);
                kfid = obj.activeFrameIDs(idx);
                
                % marginalize the frame
                graph = obj.marginalizeFrame(graph, kfid);
                return;
            end
            
            
            % TODO step 2. check the distance score of each keyframe and
            % marginalize the maximum.
            error('not ready')
            distanceScores = zeros(1, length(obj.activeFrameIDs));
            epsilon = 1e-16;
            scoreIdx = 1;
            firstIdx = iidx = graph.getFrameIndex(obj.activeFrameIDs(end));
            for id = obj.activeFrameIDs
                iidx = graph.getFrameIndex(id);
                
                d_i_1 = norm(graph.FrameContainer{iidx}.imustate.p - graph.FrameContainer{firstIdx}.imustate.p);
                
                for innerId = obj.activeFrameIDs
                    if id ~= innerId && id ~= obj.activeFrameIDs(end) && id ~= obj.activeFrameIDs(end-1)
                        jidx = graph.getFrameIndex(id);
                        
                        d_i_j = norm(graph.FrameContainer{iidx}.imustate.p - graph.FrameContainer{jidx}.imustate.p);
                        
                        distanceScores(scoreIdx) = distanceScores(scoreIdx) + 1/(d_i_j + epsilon);
                        
                    end
                end
                
                distanceScores(scoreIdx) = sqrt(d_i_1) * distanceScores(scoreIdx);
                
                scoreIdx = scoreIdx + 1;
            end
            
            distanceScores
            
            [maximum, idx] = max(distanceScores);
            
            fprintf('Keyframe %i maximizes the distance score, marginilizing it.\n', obj.activeFrameIDs(idx));
            
            graph = obj.marginalizeFrame(graph, obj.activeFrameIDs(idx));
            return;
            
        end
        
        % removes the oldest frame & its landmarks from the window and approximates its
        % information with a quadratic error (prior)
        function [graph] = marginalizeFrame(obj, graph, frameID)
            
            fprintf('marginalizing frame: %i and its landmarks\n', frameID);
            
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

