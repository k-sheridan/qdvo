function [newFeatures] = detectFeatures(I, cameraModel, currentFeatures, numFeaturesDesired, gridSize, maxIntensity)
%select new features from a given image. Attempt to find new
%features in areas with no tracked features. Do not extract features where
%the mask is ~0 (from vignette). Spatially sample features.
% feature vector format: [[x1;y1], [x2;y2],...]

% n = number of desired features.
% gridSize = dimensions of the grid used for local adaptive thresholding.

% use a DSO like feature detection method with locally adaptive
% thresholding.

fprintf('Looking for %i new features\n', numFeaturesDesired);

s = Settings();

invariantThreshold = 0.1; % the magnitude must be > 50% between the mean and max
absoluteMinGrad = s.minimumNormalizedGradientMagnitude * maxIntensity; % the absolute minumum gradient magnitude 
spatialSamplingRadius = s.featureSeparation; % the manhattan distance between features
medianFilterSize = s.medianFilterSize; % this is the size of the median filter kernel
structureTensorRadius = 3; % the radius used to compute the structure tensor at a pixel.
harrisK = 0.05; % the constant inside the harris score.
edgeWeight = 0.5; % value from [0, 1] determines how much we want edges extracted. if 0 only corners are detected. if 1 edges and corners are equially good.

% compute grid spacing.
[m, n] = size(I);
rowSpacing = floor(m/gridSize);
colSpacing = floor(n/gridSize);

newFeatures = [];

averageFeaturesPerGridSection = ceil(numFeaturesDesired / gridSize^2);

% smooth image with median filter.
K = medfilt2(I, [medianFilterSize, medianFilterSize]);

% initialize the spatial mask with the already detected features.
% Additionally, count the number of features in each box;
mask = zeros(m, n);
featureCountGrid = zeros(gridSize, gridSize);

% determine how many features are in each grid section and fill mask
for index = (1:length(currentFeatures))
    px = floor(currentFeatures(1:2, index));
    gridLocation = floor(px./[rowSpacing; colSpacing]);
    gridLocation = max(min(gridLocation, gridSize),1);
    
    mask(max(px(1)-spatialSamplingRadius, 1):min(px(1)+spatialSamplingRadius, m), max(px(2)-spatialSamplingRadius, 1):min(px(2)+spatialSamplingRadius, n)) = 1;
    
    featureCountGrid(gridLocation(1), gridLocation(2)) = featureCountGrid(gridLocation(1), gridLocation(2)) + 1;
end


% compute the image gradients
[gradX, gradY] = gradient(K);

magGrad = sqrt((gradX.^2) + (gradY.^2));

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
            / (gradMaxGrid(gridRow, gridCol) - gradMeanGrid(gridRow, gridCol))) > invariantThreshold)...
            & magGrad(ml:mu, nl:nu) > absoluteMinGrad;
        
    end
end

% select features with nonmax suppression (rank by harris score) and spatial sampling.

% Harris Score = h = det(S) - k*trace(S), S = structure tensor are pixel.
% |h| = small -> no texture
% h <<<<<< 0 -> strong edge
% h >>>>>> 0 -> strong corner

for gridRow = (1:gridSize)
    for gridCol = (1:gridSize)
        
        % compute how many features are needed in this grid section
        featureDeficit = averageFeaturesPerGridSection - featureCountGrid(gridRow, gridCol);
        
        % compute and sort the scores of all potential features in this
        % gridsection
        
        nl = max((gridCol-1)*colSpacing+1, structureTensorRadius+1);
        nu = (gridCol)*colSpacing;
        ml = max((gridRow-1)*rowSpacing+1, structureTensorRadius+1);
        mu = (gridRow)*rowSpacing;
        
        % check if near upper bounds
        if ((m - mu) < rowSpacing)
            mu = m-structureTensorRadius;
        end

        if ((n - nu) < colSpacing)
            nu = n-structureTensorRadius;
        end
        
        numCandidates = sum(sum(thresholdedGrads(ml:mu, nl:nu)));
        
        candidateArray = []; % use this to store pixel locations and harris scores. [x, y, score; x, y, score;]
        
        for imageRow = (ml:mu)
            for imageCol = (nl:nu)
                
                if ~mask(imageRow, imageCol) && thresholdedGrads(imageRow, imageCol)
                    gradXRegion = gradX(imageRow-structureTensorRadius:imageRow+structureTensorRadius,...
                        imageCol-structureTensorRadius:imageCol+structureTensorRadius);
                    gradYRegion = gradY(imageRow-structureTensorRadius:imageRow+structureTensorRadius,...
                        imageCol-structureTensorRadius:imageCol+structureTensorRadius);
                    
                    dxdx = sum(sum(gradXRegion.*gradXRegion));
                    dydy = sum(sum(gradYRegion.*gradYRegion));
                    dxdy = sum(sum(gradXRegion.*gradYRegion));
                    
                    detS = dxdx*dydy - dxdy*dxdy;
                    traceS = dxdx + dydy;
                    
                    harris = detS - harrisK * traceS^2;
                    
                    score = (harris < 0)*-edgeWeight*harris + (harris >= 0)*harris;
                    
                    candidateArray = [candidateArray; [imageRow, imageCol, score]];
                    
                end
                
            end
        end
        
        % sort the candidate array and spatially sample. This is the most
        % expensive but best method of spatial sampling.
        if (length(candidateArray) > 0)
        
            candidateArray = sortrows(candidateArray, 3, 'descend');
            [candidateRow, candidateCol] = size(candidateArray);
            
            % final step of spatial sampling these candidates
            for index = (1:candidateRow)
                rc = candidateArray(index, 1:2)';
                if candidateArray(index, 3) <= 0
                    continue;
                end
                
                if (~mask(rc(1), rc(2))) % if this area is not masked out
                    
                    
                    newFeatures = [newFeatures, [rc(2); rc(1)]]; % flip back to x, y
                    
                    featureDeficit = featureDeficit - 1; % decrement the feature deficit
                    
                    % apply mask to this region
                    mask((max(rc(1)-spatialSamplingRadius, 1):min(rc(1)+spatialSamplingRadius, m)), (max(rc(2)-spatialSamplingRadius, 1):min(rc(2)+spatialSamplingRadius, m)))...
                    = 1;
                
                    if featureDeficit <= 0
                        break;
                    end
                end
            end
           
        end
        
        if length(newFeatures) >= numFeaturesDesired
            return;
        end
    end
end


end

