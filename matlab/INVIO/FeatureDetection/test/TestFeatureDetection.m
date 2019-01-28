load('datasets/testData1.mat')

settings = Settings();

f1 = figure
f2 = figure

for frame = testImageBuffer
    figure(f1)
    f = detectFeatures(double(frame{1}.raw_image), frame{1}.cameraModel, [], 1000, 50);
    figure(f2)
    imshow(frame{1}.raw_image)
    hold on
    scatter(f(2, :), f(1, :), 'rx')
    drawnow
end