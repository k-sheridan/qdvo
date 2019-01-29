% This generates a high quality visualization of the GMM for feature
% correspondences

load('testData1.mat');

settings = Settings();

matcher = ZNCCPatchMatcher(settings);

% test features in first frame
features = [[93; 772], [294; 403], [417; 450], [708; 405], [516; 564], [884; 901], [625; 662], [552; 346]];

fig = figure;
fig.Renderer='Painters';

subplot('Position', [0.03, 0.04, 0.47, 0.94])

tgtKf = KeyFrame();
tgtKf.frame =testImageBuffer{5};


[imgrow, imgcol] = size(tgtKf.frame.raw_image)

mainImage = imagesc(tgtKf.frame.raw_image, [0, 2^16]);
colormap gray
set(gca, 'visible', 'on')
set(gca,'TickDir','out'); % The only other option is 'in'
%daspect([1, 1, 1])

searchRadius = 20;
patchRadius = 5;

hold on
for index = (1:length(features))
    rectangle('Position', [features(1, index)-searchRadius, features(2, index)-searchRadius, 2*searchRadius+1, 2*searchRadius+1],...
        'EdgeColor','g', 'LineWidth', 0.25)
    
    text(features(1, index), features(2, index), int2str(index), 'Color', [0, 1, 0], 'HorizontalAlignment', 'center')
    
end


patches = {};
%extract patches from frame 1.
increment = 0;


for px = features
    patches{end+1} = patchFromImage(testImageBuffer{1}.raw_image, px, patchRadius);
    
    plotDim = 0.108;
    startX = 0.5;
    spacing = 0.005;
    
    ypos = 0.04 + (plotDim+spacing)*increment;
    
    subplot('Position', [startX+(plotDim/4), ypos, plotDim/2, plotDim])
    ptch = imagesc([px(1)-patchRadius, px(1)+patchRadius], [px(2)-patchRadius, px(2)+patchRadius], patches{end}.image, [0, 2^16])
    if (increment == 7)
        title('Patch')
    end
    text(px(1)-patchRadius+1, px(2)-patchRadius+1, int2str(increment+1), 'Color', [0, 1, 0], 'HorizontalAlignment', 'center')
    
    
    %set(gca, 'visible', 'off')
    set(gca,'xtick',[],'ytick',[])
    %daspect([1, 1, 1])
    
    
    % evaluate zncc and plot
    [res, scoreArray] = matcher.pixelLevelWindowedSearch(patches{end}, px, tgtKf, searchRadius);
    
    % plot
    s = subplot('Position', [startX+(2 * plotDim/4) + plotDim/2, ypos, plotDim/2, plotDim])
    zncc = imagesc([px(1)-searchRadius, px(1)+searchRadius], [px(2)-searchRadius, px(2)+searchRadius], scoreArray, [-1, 1])
    if (increment == 7)
        title('ZNCC')
    end
    colormap(gca,'default')
    
    %set(gca, 'visible', 'off')
    set(gca,'xtick',[],'ytick',[])
    %daspect([1, 1, 1])
    
    
    % plot gmms
    %thresholds = [0.9, 0.8, 0.5, 0, -1]
    thresholds = [0.9, 0.8, 0.5, 0, -0.5]
    %thresholds = [0.9]
    for index = (1:length(thresholds))
        
        information = inv([1,0;0,1]);
        
        % compute the gmm at this threshold
        [m,n] = size(scoreArray);
        syms x y
        gmm = 0*x + 0*y + 1;
        weightSum = 0;
        for row = (1:m)
            for col = (1:n)
                if (scoreArray(row, col) > thresholds(index))
                    weight = scoreArray(row, col) - thresholds(index); % ensure it is positive
                    weightSum = weightSum + weight;
                    
                    z = [px(1)-searchRadius + col; imgrow - (px(2)-searchRadius + row)];
                    
                    gmm = gmm + weight * exp(-0.5 * ([x;y] - z)' * information * ([x;y] - z));
                    
                end
            end
        end
        gmm = gmm / weightSum;
        
        subplot('Position', [startX+(3 * plotDim/4) + (index+1) * plotDim/2, ypos, plotDim/2, plotDim])
        fc = fcontour((gmm), [px(1)-searchRadius, px(1)+searchRadius, imgrow - px(2)-searchRadius, imgrow - px(2)+searchRadius], 'Fill','on');
        %fc.LevelList = (1e-20:1e-3:1)
        colormap(gca,'parula')
        set(gca,'xtick',[],'ytick',[])
        if (increment == 7)
            title(sprintf('\\Theta = %.1f', thresholds(index)))
        end
        disp('ding')
        disp(index)
    end
    
    
    
    increment = increment + 1;
    
end

