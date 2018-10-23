classdef ZNCCPatchMatcher
    %PATCHMATCHER Warps and matches a patch in a new image. This matcher
    %will return a subpixel match.
    
    properties
        warpedSrcPatch % warped patch (template)
        scoreArray % 2D array of scores (ZNCC), piexl: [x, y, score]
    end
    
    methods
        function [result] = search(obj, landmark, graph, targetKeyframe, radius)
            % perform a pixel level ZNNC search followed by a quadratic fit
            % to estimate the subpixel match result.
            
            % warp the patch into the target frame
            
            % search for best match in target frame (pixel resolution)
        end
        
        function [] = pixelLevelWindowedSearch(obj, warpedTemplate, centerPixel, targetKeyframe, searchRadius)
            % evaluates a window around the center pixel with a ZNCC
            
            % initialize the score array
            obj.scoreTensor = {-1 * ones(2*searchRadius + 1, 2*searchRadius + 1), ...
                -1 * ones(2*searchRadius + 1, 2*searchRadius + 1), -1 * ones(2*searchRadius + 1, 2*searchRadius + 1)};
            
            [patchRows, patchCols] = size(warpedTemplate.image);
            
            assert(patchRows == patchCols);
            assert(mod(patchRows, 2) == 1);
            patchRadius = (patchRows - 1) / 2;
            
            for dx = (-searchRadius:1:searchRadius)
                for dy = (-searchRadius:1:searchRadius)
                    topLeftRow = centerPixel(2) + dy - patchRadius;
                    topLeftCol = centerPixel(1) + dx - patchRadius;
                    
                    % check that indices are out not of bounds.
                    [imgRows, imgCols] = size(targetKeyframe.frame.raw_image);
                    
                    scoreArrayRow = dy + searchRadius + 1;
                    scoreArrayCol = dx + searchRadius + 1;
                    
                    if (topLeftRow < 1 || topLeftCol < 1 || topLeftRow + 1 + 2 * patchRadius > imgRows...
                            || topLeftCol + 1 + 2 * patchRadius > imgCols)
                        % if this patch is out of bounds, set its score to
                        % -1 (anti-correlated)
                        obj.scoreArray{1}(scoreArrayRow, scoreArrayCol) = centerPixel(2) + dy; % row
                        obj.scoreArray{2}(scoreArrayRow, scoreArrayCol) = centerPixel(2) + dx; % col
                        obj.scoreArray{3}(scoreArrayRow, scoreArrayCol) = -1;
                    end
                    
                    % create the patch from the target image
                    tgtPatch = Patch(targetKeyframe.frame.raw_image(topLeftRow:topLeftRow+(patchRows), topLeftCol:topLeftCol+(patchCols)));
                    
                    % evaluate the zncc for this patch
                    score = obj.zncc(warpedTemplate, tgtPatch)
                    
                    % set the score in the score array
                    obj.scoreArray{1}(scoreArrayRow, scoreArrayCol) = centerPixel(2) + dy; % row
                    obj.scoreArray{2}(scoreArrayRow, scoreArrayCol) = centerPixel(2) + dx; % col
                    obj.scoreArray{3}(scoreArrayRow, scoreArrayCol) = score;
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

