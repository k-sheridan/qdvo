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
        
    end
    
    methods
        
        function [gaussianWeights] = computeGaussianWeightsRobustly(obj, px, theta)
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
                        
                        gaussianWeights(i) = gaussianWeights(i) + ...
                            ((obj.potentialCorrespondenceSet{j}.score - theta) / (obj.potentialCorrespondenceSet{i}.score - theta)) * ...
                            exp(-(px - uj)'*(px - uj) + (px - ui)'*(px - ui));
                    end
                end
                
                gaussianWeights(i) = 1 / gaussianWeights(i);
                
            end
        end
        
        function [g] = evaluateUnnormalizedGaussian(obj, pc, px)
            % computes the exponential part of the gaussian distribution
            % for a given px and correspondence.
            error = (px - pc.pixel);
            
            g = exp(-error' * error); % this is true because the cov = eye(2)
        end
        
        function [cov] = computeGMMCovariance(obj)
            % this function computes the overall uncertainty in the gmm.
            
        end
    end
end

