% tests the zncc patch matcher
load('/Users/kevinsheridan/Documents/Mac Library/RnD/IARC - Mission 7/invio/octave/INVIO/FeatureTracking/test/testData1.mat');

settings = Settings();

matcher = ZNCCPatchMatcher(settings);

% test features in first frame
features = [[398; 452], [653; 398], [532; 791], [551; 346], [801; 488], [806; 386]];

figure('Name', 'Frame 1 with initial features')
imshow(testImageBuffer{1}.raw_image)
hold on
scatter(features(1, :), features(2, :))

patches = {};
%extract patches from frame 1.
for px = features
    patches{end+1} = patchFromImage(testImageBuffer{1}.raw_image, px, settings.patchHalfSize);
end


trackedFeatures = features;
figure('Name', 'Frame n with initial features')
resultArray = {};

% search for the patch in frame 1
for frame = testImageBuffer
    tgtKf = KeyFrame();
    tgtKf.frame = frame{1};
    clf;
    imshow(frame{1}.raw_image);
    
    for index = (1:length(trackedFeatures))
        res = matcher.pixelLevelWindowedSearch(patches{index}, trackedFeatures(1:2, index), tgtKf, 7);
        trackedFeatures(1:2, index) = res.pixel;
        resultArray{index} = res;
    end
    
    % draw the results
    for res = resultArray
        if (res{1}.error == MatchError.NONE)
            hold on
            scatter(res{1}.pixel(1), res{1}.pixel(2), 'go');
        else
            hold on
            scatter(res{1}.pixel(1), res{1}.pixel(2), 'ro');
        end
    end
    
    pause(1)
end






