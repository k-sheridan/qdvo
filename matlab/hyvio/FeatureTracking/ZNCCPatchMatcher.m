classdef ZNCCPatchMatcher < handle
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
        
        function [result] = search(obj, landmark, sourceKeyFrame, targetKeyframe, searchRadius)
            % perform a pixel level ZNNC search followed by a quadratic fit
            % to estimate the subpixel match result.
            
            % warp the patch into the target frame
            [warpedTemplatePatch] = warpPatchToTargetFrame(landmark, sourceKeyFrame, targetKeyframe, obj.settings.patchHalfSize);
            
            % compute the point to search around in the target image given
            % its pose and the source pose.
            %P_tgt = inv(T_tgt) * T_src * P_src
            lmPos_tgt = inv(targetKeyframe.imustate.poseTransform()) * sourceKeyframe.imustate.poseTransform() * ([landmark.bearing;1] / landmark.zinv);
            [centerPixel, projJac] = targetKeyframe.cameraModel.project(lmPos_tgt);
            
            % search for best match in target frame (pixel resolution)
            [result, scoreArray] = obj.pixelLevelWindowedSearch(warpedTemplatePatch, centerPixel, targetKeyframe, searchRadius);
            
            % compute sub pixel estimate.
        end
        
        function [result, scoreArray] = pixelLevelWindowedSearch(obj, srcPatch, centerPixel, targetKeyframe, searchRadius)
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
                    tgtPatch = patchFromImage(targetKeyframe.raw_image, patchCenter, patchRadius);
                    
                    % compare the two patches
                    score = obj.zncc(srcPatch, tgtPatch);
                    
                    % add the score to the array
                    obj.scoreArray(dy + searchRadius + 1, dx + searchRadius + 1) = score;
                end
            end
            
            scoreArray = obj.scoreArray;
            
            % find the best match.
            maxRow = 1;
            maxCol = 1;
            [scores, rows] = max(obj.scoreArray);
            [score, col] = max(scores);
            maxRow = rows(col);
            maxCol = col;
            
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
            
            
            % TODO now ensure that the match is suficiently unique
            if (obj.settings.correlationUniquenessThreshold > 0) 
                % TODO
            end
                
            
        end
        
        function [score] = zncc(obj, templatePatch, targetPatch)
            % computes the ZNCC of these two patches.
            % WARNING uniform patch causes instability. DO NOT give a
            % uniform template patch. It is assummed that the template is
            % textured.
            
            % one liner: sum(((A-mean(A)).*(B-mean(B))))/sqrt(sum((A-mean(A)).^2 * sum((B-mean(B)).^2)))
            
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

