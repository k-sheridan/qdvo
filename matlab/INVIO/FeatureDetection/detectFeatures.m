function [newFeatures] = detectFeatures(I, cameraModel, currentFeatures, n, gridSize)
%select new features from a given image. Attempt to find new
%features in areas with no tracked features. Do not extract features where
%the mask is ~0 (from vignette). Spatially sample features.
% feature vector format: [[x1;y1], [x2;y2],...]

% n = number of desired features.
% gridSize = dimensions of the grid used for local adaptive thresholding.

% use a DSO like feature detection method with locally adaptive
% thresholding.

invariantThreshold = 0.1; % the magnitude must be > 50% between the mean and max

% compute grid spacing.
[m, n] = size(I);
rowSpacing = floor(m/gridSize);
colSpacing = floor(n/gridSize);

% smooth image with median filter.
K = medfilt2(I);

% compute the image gradients
[gradX, gradY] = gradient(K);

magGrad = sqrt((gradX.^2).*(gradY.^2));

% adaptive threshold with grid
gradMeanGrid = zeros(gridSize);
gradMaxGrid = zeros(gridSize);

thresholdedGrads = zeros(m, n);

for gridRow = (1:gridSize)
    for gridCol = (1:gridSize)
        
        nl = (gridCol-1)*colSpacing+1;
        nu = (gridCol)*colSpacing;
        ml = (gridRow-1)*rowSpacing+1;
        mu = (gridRow)*rowSpacing;
        
        % check if near upper bounds
        if ((m - mu) < rowSpacing)
            mu = m;
        end

        if ((n - nu) < colSpacing)
            nu = n;
        end
        
        gradMeanGrid(gridRow, gridCol) = mean(mean(magGrad(ml:mu, nl:nu)));
        gradMaxGrid(gridRow, gridCol) = max(max(magGrad(ml:mu, nl:nu)));
        
        thresholdedGrads(ml:mu, nl:nu) = (((magGrad(ml:mu, nl:nu) - gradMeanGrid(gridRow, gridCol))...
            / (gradMaxGrid(gridRow, gridCol) - gradMeanGrid(gridRow, gridCol))) > invariantThreshold);%.* magGrad(ml:mu, nl:nu);
        
    end
end

imagesc(thresholdedGrads)

end

