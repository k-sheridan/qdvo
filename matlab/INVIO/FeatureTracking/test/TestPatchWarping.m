% test the patch warp function
load('testData1.mat');

settings = Settings();

matcher = ZNCCPatchMatcher(settings);

% test features in first frame
feature = [210; 452];

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

figure
imshow(tgtkf.frame.raw_image)
figure
for delta = (0:0.01:1)
% set a new pose for the tgt frame
tgtkf.frame.imustate.r = [delta * 5;0;delta * -3]
tgtkf.frame.imustate.q = eul2quat([0,0 delta * pi/6])'
    
patch = warpPatchToTargetFrame(lm, srckf, tgtkf, 20);

image(patch.image/2^16 * 255)
drawnow
end
