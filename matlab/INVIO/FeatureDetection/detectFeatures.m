function [newFeatures] = detectFeatures(I, cameraModel, currentFeatures, n, gridSize)
%select new features from a given image. Attempt to find new
%features in areas with no tracked features. Do not extract features where
%the mask is ~0 (from vignette). Spatially sample features.
% feature vector format: [[x1;y1], [x2;y2],...]

% n = number of desired features.
% gridSize = dimensions of the grid used for local adaptive thresholding.

% use a DSO like feature detection method with locally adaptive
% thresholding.

% compute grid spacing.
[m, n] = size(image);
rowSpacing = floor(m/gridSize)
colSpacing = floor(n/gridSize)

% smooth image.
%K = imgaussfilt(double(I),4);

% compute the image gradients
[gradX, gradY] = gradient(I);

%imshow(sqrt(gradX.*gradY))

end

