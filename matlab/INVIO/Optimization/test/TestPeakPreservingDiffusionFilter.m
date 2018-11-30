% This is a test at a function which will smooth the quasi convex ZNCC
% function

load('FeatureTracking/test/testData1.mat')

settings = Settings();

f1 = figure('Name', 'Template')
colormap gray

imagesc(template.image, [0, 2^16])

f2 = figure('Name', 'Result')
colormap gray

tgtImage = testImageBuffer{10}.raw_image;

imagesc(testImageBuffer{10}.raw_image, [0, 2^16])

[m, n] = size(tgtImage);

kernelSize = 21;
patchSize = 11;
patchRadius = (patchSize-1)/2;
kernelRadius = (kernelSize-1)/2;

maxSigma = 25;
sigmoidConst = 3;

maxRadius = max(patchRadius, kernelRadius);

template = patchFromImage(testImageBuffer{1}.raw_image, [293, 431]', patchRadius)

znccImage = -ones(m, n);

matcher = ZNCCPatchMatcher(settings);

for row = (maxRadius+1:(m-maxRadius))
    for col = (maxRadius+1:(n-maxRadius))
        % evaluate zncc
        znccImage(row, col) = matcher.zncc(template, patchFromImage(tgtImage, [col, row]', patchRadius));
    end
end

f3 = figure('Name', 'ZNCC Raw')
imagesc(znccImage, [-1, 1])
colormap jet
colorbar

znccFilteredImage = -ones(m, n);

nc = (1 / (1 + exp(-sigmoidConst))) - (1 / (1 + exp(sigmoidConst)));
minSig = (1 / (1 + exp(sigmoidConst)));

for row = (maxRadius+1:(m-maxRadius))
    for col = (maxRadius+1:(n-maxRadius))
        % run filter
        sigma = maxSigma/nc * (1 / (1 + exp(sigmoidConst*znccImage(row, col))) - minSig) + 1e-8;
        kernel = fspecial('gaussian',kernelSize,sigma);
        
        patch = znccImage(row-kernelRadius:row+kernelRadius, col-kernelRadius:col+kernelRadius);
        
        znccFilteredImage(row, col) = sum(sum(kernel.*patch));
    end
end


f4 = figure('Name', 'ZNCC Filtered')
imagesc(znccFilteredImage, [-1, 1])
colormap jet
colorbar