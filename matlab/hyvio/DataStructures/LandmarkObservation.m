classdef LandmarkObservation < handle
    %OBSERVATION a set of potential correspondences between a landmark and
    %image. This is used to compute a GMM correspondence distribution. It
    %is a hybrid between an indirect and direct method.
    %
    % each potential correspondence has an associated score.
    % ASSUMES: each correspondence has a variance of 1! (this allows for quicker computation, and makes sense)
    
    properties
        potentialCorrespondenceSet = {} % the set of all potential correspondences for a landmark
        
        theta % the threshold used for matching
        observationFrameID % id of observation keyframe in pose graph.
        landmarkParentFrameID % id of landmark keyframe in pose graph.
        landmarkID % id of landmark in keyframe.
        
        searchPatch; % for debugging.
        
    end
    
    methods
        
        function [gaussianWeights] = computeGaussianWeightsRobustly(obj, px)
            % computes the inverse gaussian weights which avoids numerical
            % issues
            n = length(obj.potentialCorrespondenceSet);
            gaussianWeights = zeros(1, n);
            
            for i = (1:n)
                for j = (1:n)
                    if i == j
                        gaussianWeights(i) = gaussianWeights(i) + 1;
                    else
                        ui = obj.potentialCorrespondenceSet{i}.pixel;
                        uj = obj.potentialCorrespondenceSet{j}.pixel;
                        
                        e = exp(-1/2*(px - uj)'*(px - uj) + 1/2*(px - ui)'*(px - ui));
                        w = ((obj.potentialCorrespondenceSet{j}.score) / (obj.potentialCorrespondenceSet{i}.score));
                        
                        gaussianWeights(i) = gaussianWeights(i) + w * e;
                    end
                end
                
                
                gaussianWeights(i) = 1 / gaussianWeights(i);
                
            end
        end
        
        % Computes the weighted error of the form: (px - z_i)
        % assumes the cov of each potential correspondence is 1 px^2
        function [residual] = computeResidual(obj, px)
            n = length(obj.potentialCorrespondenceSet);
            errorArray = zeros(2, n); % an array of column vectors for each PC.
            scoreArray = zeros(1, n);
            idx = 1;
            for pc = obj.potentialCorrespondenceSet
                errorArray(1:2, idx) = px - pc{1}.pixel;
                scoreArray(1, idx) = pc{1}.score;
                idx = idx + 1;
            end
            
            sqErrorArray = -1/2*sum((errorArray.^2));
            
            expArr_score = exp(sqErrorArray).*scoreArray;
            
            GMM = sum(expArr_score);
            
            if GMM < realmin
                error('cannot compute the reidual! error too large.')
            end
            
            weights = expArr_score / GMM;
            
            residual = (errorArray.*weights);
            residual = [sum(residual(1,:)); sum(residual(2,:))];
        end
        
        function [g] = evaluateUnnormalizedGaussian(obj, pc, px)
            % computes the exponential part of the gaussian distribution
            % for a given px and correspondence.
            error = (px - pc.pixel);
            
            g = exp(-1/2*error' * error); % this is true because the cov = eye(2)
        end
        
        function [cov, mean] = computeGMMCovariance(obj)
            % this function computes the overall uncertainty in the gmm.
            pxSum = [0;0];
           
            
            
            
            for pc = obj.potentialCorrespondenceSet
                pxSum = pxSum + pc{1}.pixel;
            end
            
            mean = pxSum / length(obj.potentialCorrespondenceSet);
            
            cov = eye(2);
            
            for pc = obj.potentialCorrespondenceSet
                error = pc{1}.pixel - mean;
                cov = cov + (pc{1}.score) * (error*error');
            end
        end
    end
end

