function [intensity] = subPixelIntensity1(px, image)
% sub pixel intensity using bilinear interpolation. returns brightness and
% if it passed. pixel is in [x; y]

% top left index
x1_y1 = floor(px);

x1 = x1_y1(1);
x2 = x1 + 1;
y1 = x1_y1(2);
y2 = y1 + 1;

A = double([image(y1, x1), image(y1, x2); image(y2, x1), image(y2, x2)]);

intensity = [y2 - px(2), px(2) - y1] * A * [x2 - px(1); px(1) - x1];

end

