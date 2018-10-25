function [patch] = subPixelPatchFromImage(image, centerPixel, patchRadius)
%PATCHFROMIMAGE create a patch from an image around a pixel

data = zeros(patchRadius * 2 + 1);

for row = (1:(2*patchRadius + 1))
    for col = (1:(2*patchRadius + 1))
        % use bilinear interpolation to get value between pixels
        data(row, col) = subPixelIntensity1(centerPixel + [col - 1 - patchRadius; col - 1 - patchRadius], image);
    end
end

% TODO check that the bounds are feasible

patch = Patch(data);
end

