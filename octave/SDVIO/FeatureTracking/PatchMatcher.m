classdef PatchMatcher
    %PATCHMATCHER Warps and matches a patch in a new image. This matcher
    %will return a subpixel match.
    
    properties
        warpedSrcPatch % warped patch (template)
        scoreArray % 2D array of scores
    end
    
    methods
        function [result] = search(landmark, graph, targetKeyframe)
            % perform a pixel level ZNNC search followed by a quadratic fit
            % to estimate the subpixel match result.
        end
    end
end

