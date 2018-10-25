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


% search for the patch in frame 1
tgtKf = KeyFrame();
tgtKf.frame = testImageBuffer{1};
matcher.pixelLevelWindowedSearch(patches{1}, features(1:2, 1), tgtKf, 20)




