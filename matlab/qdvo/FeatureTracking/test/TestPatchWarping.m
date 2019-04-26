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

srckf = Frame(testImageBuffer{1}.raw_image, 0, IMUState());
srckf.id = 1;
srckf.cameraModel = cameraModel;

tgtkf = Frame(testImageBuffer{80}.raw_image, 0, IMUState());
tgtkf.id = 2;
tgtkf.cameraModel = cameraModel;

figure
imshow(tgtkf.raw_image)
figure
for delta = (0:0.01:1)
% set a new pose for the tgt frame
tgtkf.imustate.p = [delta * 5;0;delta * -3];
tgtkf.imustate.R = eul2rotm([0,0 delta * pi/6])';
    
patch = warpPatchToTargetFrame(lm, srckf, tgtkf, 20);

colormap gray
imagesc(patch.image, [0, 2^16])
drawnow
end
