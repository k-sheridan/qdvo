function [intensity, pass] = subPixelIntensity1(px, image)
% sub pixel intensity using bilinear interpolation. returns brightness and
% if it passed. pixel is in [x; y]

% top left index
px = circshift(px, 1); % flip into indices
x1_y1 = floor(px);

x1 = x1_y1(1);
x2 = x1 + 1;
y1 = x1_y1(2);
y2 = y1 + 1;

A = double(image(x1_y1(2):(x1_y1(2)+1), x1_y1(1):(x1_y1(1)+1)));



intensity = [x2 - px(1), px(1) - x1] * A * [y2 - px(2); px(2) - y1];


end

