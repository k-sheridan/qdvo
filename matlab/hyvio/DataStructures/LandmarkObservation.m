classdef LandmarkObservation < handle
    %OBSERVATION a set of potential correspondences between a landmark and
    %image. This is used to compute a GMM correspondence distribution. It
    %is a hybrid between an indirect and direct method.
    %
    % each potential correspondence has an associated score.
    % ASSUMES: each correspondence has a variance of 1! (this allows for quicker computation, and makes sense)
    
    properties
        potentialCorrespondenceSet = {} % the set of all potential correspondences for a landmark
        
        observationFrameID % id of observation keyframe in pose graph.
        landmarkParentFrameID % id of landmark keyframe in pose graph.
        landmarkID % id of landmark in keyframe.
        
        cov;
        
    end
    
    methods
        
        function [gaussianWeights] = computeGaussianWeights(obj, px, theta)
            % theta: score threshold, px: evaluate around this pixel.
            
            n = length(obj.potentialCorrespondenceSet);
            indexArray = (1:n);
            gaussianWeights = zeros(1, n);
            
            scoreSum = 0;
            for idx = indexArray
                scoreSum = scoreSum + obj.potentialCorrespondenceSet{idx}.score;
            end
            scoreSum = scoreSum - n*theta;
            
            % compute the weighted gaussians of each potential
            % correspondence
            for idx = indexArray
                gaussianWeights(idx) = ((obj.potentialCorrespondenceSet{idx}.score - theta) / scoreSum)...
                    * obj.evaluateUnnormalizedGaussian(potentialCorrespondenceSet{idx}, px);
            end
            
            gmm = sum(gaussianWeights);
            
            % TODO check if gmm is too small
            
            % finally, compute the weights
            for idx = indexArray
                gaussianWeights(idx) = gaussianWeights(idx) / gmm;
            end
            
        end
        
        function [g] = evaluateUnnormalizedGaussian(obj, pc, px)
            % computes the exponential part of the gaussian distribution
            % for a given px and correspondence.
            error = (px - pc.pixel);
            
            g = exp(error' * error); % this is true because the cov = eye(2)
        end
        
        function [] computeGMMCovariance(obj)
            % this function computes the overall uncertainty in the gmm.
            
        end
        
    end
end

