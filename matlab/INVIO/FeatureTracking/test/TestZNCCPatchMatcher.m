% tests the zncc patch matcher
load('testData1.mat');

settings = Settings();

matcher = ZNCCPatchMatcher(settings);

% test features in first frame
features = [[398; 452], [653; 398], [532; 791], [551; 346], [801; 488], [806; 386], [93;772]];

figure('Name', 'Frame 1 with initial features')
imshow(testImageBuffer{1}.raw_image)
hold on
scatter(features(1, :), features(2, :))

patches = {};
%extract patches from frame 1.
for px = features
    patches{end+1} = patchFromImage(testImageBuffer{1}.raw_image, px, 10);
end

f2 = figure('Name', 'scores');

trackedFeatures = features;
f1 = figure('Name', 'Frame n with initial features')
resultArray = {};

% search for the patch in frame 1
for frame = testImageBuffer
    tgtKf = KeyFrame();
    tgtKf.frame = frame{1};
    
    for index = (1:length(trackedFeatures))
        [res, scoreArray] = matcher.pixelLevelWindowedSearch(patches{index}, trackedFeatures(1:2, index), tgtKf, 20);
        
        
        figure(f2)
        
        colormap gray
        
        subplot(2, 2, 1)
        imagesc((scoreArray), [-1, 1])
        colorbar
        title('ZNCC response')
        
        
        [dx, dy] = gradient((scoreArray));
        
        subplot(2, 2, 2)
        imagesc(dx)
        colorbar
        title('dzncc/dx')
        
        subplot(2, 2, 3)
        imagesc(abs(dy.*dx))
        colorbar
        title('extrema')
        
        subplot(2, 2, 4)
        imagesc(patches{index}.image / 2^16 * 255, [0, 255])
        title('Source Patch')
        
        drawnow
        %pause(1)
        
        trackedFeatures(1:2, index) = res.pixel;
        resultArray{index} = res;
    end
    
    % draw the results
    figure(f1)
    clf;
    imshow(frame{1}.raw_image);
    for res = resultArray
        if (res{1}.error == MatchError.NONE)
            hold on
            scatter(res{1}.pixel(1), res{1}.pixel(2), 'go');
        else
            hold on
            scatter(res{1}.pixel(1), res{1}.pixel(2), 'ro');
        end
    end
    
    %pause(1)
end






