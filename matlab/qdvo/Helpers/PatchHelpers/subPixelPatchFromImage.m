function [patch] = subPixelPatchFromImage(image, centerPixel, patchRadius, order)
%PATCHFROMIMAGE create a patch from an image around a pixel

data = zeros(patchRadius * 2 + 1);

%TODO add higher order interpolation
if (nargin <= 3)
    order = 1;
end

for row = (1:(2*patchRadius + 1))
    for col = (1:(2*patchRadius + 1))
        % use bilinear interpolation to get value between pixels
        data(row, col) = subPixelIntensity1([centerPixel(1) + col - 1 - patchRadius; centerPixel(2) + row - 1 - patchRadius], image);
    end
end


% TODO check that the bounds are feasible

patch = Patch(data);
end

