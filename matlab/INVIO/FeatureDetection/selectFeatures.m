function [newFeatures] = selectFeatures(image, cameraModel, currentFeatures)
%SELECTFEATURES select new features from a given image. Attempt to find new
%features in areas with no tracked features. Do not extract features where
%the mask is ~0 (from vignette). Spatially sample features.
% feature vector format: [[x1;y1], [x2;y2],...]

% use a DSO like feature detection method with locally adaptive
% thresholding.
end

