load('testData1.mat');

settings = Settings();

centerPixel = [398; 452];

p0 = subPixelPatchFromImage(testImageBuffer{1}.raw_image, centerPixel, 20);

image(p0.image / 2^16 * 255);
colormap gray
caxis([0,255])
colorbar
figure

for delta = (0:0.01:40)
    p = subPixelPatchFromImage(testImageBuffer{1}.raw_image, centerPixel + [delta;0], 20);
    image(p.image / 2^16 * 255);
    colormap gray
    caxis([0,255])
    colorbar
    drawnow
end