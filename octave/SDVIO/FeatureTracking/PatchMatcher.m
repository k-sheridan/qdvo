classdef PatchMatcher
    %PATCHMATCHER Warps and matches a patch in a new image. This matcher
    %will return a subpixel match.
    
    properties
        warpedSrcPatch % warped patch (template)
        scoreTensor % 2D array of scores (ZNCC), piexl: [x, y, score]
    end
    
    methods
        function [result] = search(obj, landmark, graph, targetKeyframe, radius)
            % perform a pixel level ZNNC search followed by a quadratic fit
            % to estimate the subpixel match result.
            
            % warp the patch into the target frame
            
            % search for best match in target frame (pixel resolution)
        end
        
        function [] = pixelLevelWindowedSearch(obj, warpedTemplate, centerPixel, targetKeyframe, radius)
            % evaluates a window around the center pixel with a ZNCC
            
            % initialize the score array
            obj.scoreTensor = {zeros(2*radius + 1, 2*radius + 1), zeros(2*radius + 1, 2*radius + 1), -1 * ones(2*radius + 1, 2*radius + 1)};
            
            [patchRows, patchCols] = size(warpedTemplate.image);
            
            for dx = (-radius:1:radius)
                for dy = (-radius:1:radius)
                    row = centerPixel(2) + dy;
                    col = centerPixel(1) + dx;
                    
                    % TODOcreate the patch from the target image
                    
                end
            end
        end
        
        function [score] = zncc(obj, templatePatch, targetPatch)
            % computes the ZNCC of these two patches.
            % WARNING uniform patch causes instability. DO NOT give a
            % uniform template patch. It is assummed that the template is
            % textured.
            assert(strcmp(class(templatePatch), 'Patch') && strcmp(class(templatePatch), 'Patch'));
            assert(isequal(size(templatePatch.image), size(targetPatch.image)))
            
            if (templatePatch.sumZeroMeanSquared <= 1e-8 || targetPatch.sumZeroMeanSquared <= 1e-8)
                warning('Uniform patch!')
                score = 0;
                return
            end
            
            [m, n] = size(templatePatch.image);
            
            score = 0;
            
            for row = (1:m)
                for col = (1:n)
                    score = score + (templatePatch.image(row, col) - templatePatch.meanIntensity) * (targetPatch.image(row, col) - targetPatch.meanIntensity);
                end
            end
            
            score = score / sqrt(templatePatch.sumZeroMeanSquared * targetPatch.sumZeroMeanSquared);
        end
    end
end

