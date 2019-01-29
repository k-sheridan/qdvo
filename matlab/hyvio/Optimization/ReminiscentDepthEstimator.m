classdef ReminiscentDepthEstimator < handle
    %REMINISCENTDEPTHESTIMATOR looks into the frame history to generate a
    %set of 1D correspondence distributions. Additionally serves as a
    %premptive outlier rejection method similar to the depth filter in SVO.
    
    % 
    properties
        windowSize; % the number of frames to look back for correspondences.
        minDepth, maxDepth; % the bounds for the search.
        resolution; % the number of discrete inverse depths to compute the likelihood for.
    end
    
    methods
        function obj = ReminiscentDepthEstimator(windowSize, minDepth, maxDepth, resolution)
            obj.windowSize = windowSize;
            obj.minDepth = minDepth;
            obj.maxDepth = maxDepth;
            obj.resolution = resolution;
        end
    end
end

