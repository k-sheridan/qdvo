function [patch] = patchFromImage(image, centerPixel, patchRadius)
%PATCHFROMIMAGE create a patch from an image around a pixel

centerPixel = floor(centerPixel); % force to integer

lx = centerPixel(1) - patchRadius;
hx = centerPixel(1) + patchRadius;
ly = centerPixel(2) - patchRadius;
hy = centerPixel(2) + patchRadius;


patch = Patch((image(ly:hy, lx:hx)));
end

