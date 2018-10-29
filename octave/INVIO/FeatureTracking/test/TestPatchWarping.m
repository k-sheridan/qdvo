% test the patch warp function
load('/Users/kevinsheridan/Documents/Mac Library/RnD/IARC - Mission 7/invio/octave/INVIO/FeatureTracking/test/testData1.mat');

settings = Settings();

matcher = ZNCCPatchMatcher(settings);

% test features in first frame
feature = [398; 452];

figure('Name', 'Frame 1 with initial features')
imshow(testImageBuffer{1}.raw_image)
hold on
scatter(feature(1, :), feature(2, :))

%
lm = Landmark();
lm.bearing = cameraModel.unproject(feature);
lm.zinv = 1/3;
lm.id = 1;
lm.kfid = 1;
lm.px = feature;
lm.zinvVariance = 1;

srckf = KeyFrame();
srckf.id = 1;
srckf.frame = testImageBuffer{1};

tgtkf = KeyFrame();
tgtkf.id = 2;
tgtkf.frame = testImageBuffer{80};



