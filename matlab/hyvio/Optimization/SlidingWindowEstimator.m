classdef SlidingWindowEstimator
    %SLIDINGWINDOWESTIMATOR this is an implementation of a sliding window
    %estimator. It handles marginalization, and optimization of a local
    %window of the graph.
    
    properties
        windowSize; % integer list how many frames into the past are optimized.
        
        inertialJacobianStorage {};
        visualJacobianStorage {};
        priorJacobianStorage {};
    end
    
    methods
        function obj = SlidingWindowEstimator(windowSize)
            obj.windowSize;
        end
        
        
    end
end

