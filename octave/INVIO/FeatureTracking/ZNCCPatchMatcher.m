classdef ZNCCPatchMatcher
    %PATCHMATCHER Warps and matches a patch in a new image. This matcher
    %will return a subpixel match.
    
    properties
        warpedSrcPatch % warped patch (template)
        scoreArray % 2D array of scores (ZNCC):
        settings
    end
    
    methods
        function [obj] = ZNCCPatchMatcher(settings)
            % initialize the patch matcher with a settings struct.
            obj.settings = settings;
        end
        
        function [result] = search(obj, landmark, graph, targetKeyframe, radius)
            % perform a pixel level ZNNC search followed by a quadratic fit
            % to estimate the subpixel match result.
            
            % warp the patch into the target frame
            
            % search for best match in target frame (pixel resolution)
        end
        
        function [result] = pixelLevelWindowedSearch(obj, srcPatch, centerPixel, targetKeyframe, searchRadius)
            % evaluates a window around the center pixel with a ZNCC
            
            obj.scoreArray = -1 * ones(2*searchRadius + 1);
            
            [patchRows, patchCols] = size(srcPatch.image);
            
            assert(patchRows == patchCols);
            assert(mod(patchRows, 2) == 1);
            patchRadius = (patchRows - 1) / 2;
            
            for dx = (-searchRadius:1:searchRadius)
                for dy = (-searchRadius:1:searchRadius)
                    patchCenter = centerPixel + [dx;dy];
                    
                    % create patch in tgt image
                    tgtPatch = patchFromImage(targetKeyframe.frame.raw_image, patchCenter, patchRadius);
                    
                    % compare the two patches
                    score = obj.zncc(srcPatch, tgtPatch);
                    
                    % add the score to the array
                    obj.scoreArray(dy + searchRadius + 1, dx + searchRadius + 1) = score;
                end
            end
            
            % find the best match.
            [m, n] = size(obj.scoreArray);
            maxRow = 1;
            maxCol = 1;
            for row = (1:m)
                for col = (1:n)
                    if (obj.scoreArray(row, col) > obj.scoreArray(maxRow, maxCol))
                        maxRow = row;
                        maxCol = col;
                    end
                end
            end
            
            result = MatchResult();
            result.pixel = [maxCol - 1 - searchRadius; maxRow - 1 - searchRadius] + centerPixel;
            result.covariance = eye(2) * 25;
            result.zncc = obj.scoreArray(maxRow, maxCol);
            
            % ensure match is good enough.
            if (obj.scoreArray(maxRow, maxCol) < obj.settings.minimumNormalizedMatchCorrelation)
                % match failed
                result.error = MatchError.MATCH_SCORE_BELOW_THRESHOLD;
                return
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
            
            score = score / sqrt(double(templatePatch.sumZeroMeanSquared * targetPatch.sumZeroMeanSquared));
        end
    end
end

