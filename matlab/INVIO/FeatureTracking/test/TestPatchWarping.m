% test the patch warp function
load('testData1.mat');

settings = Settings();

matcher = ZNCCPatchMatcher(settings);

% test features in first frame
feature = [200; 452];

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
srckf.frame.imustate = IMUState();
srckf.frame.cameraModel = cameraModel;

tgtkf = KeyFrame();
tgtkf.id = 2;
tgtkf.frame = testImageBuffer{80};
tgtkf.frame.imustate = IMUState();
tgtkf.frame.cameraModel = cameraModel;

patch = warpPatchToTargetFrame(lm, srckf, tgtkf, 20);



