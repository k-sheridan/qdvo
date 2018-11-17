load('FeatureTracking/test/testData1.mat')

settings = Settings();

for frame = testImageBuffer
    detectFeatures(double(frame{1}.raw_image), frame{1}.cameraModel, [], 1000, 20);
    drawnow
end