function [grad] = computeImageGradient(image, pixel)
% Compute the image gradient around a pixel with a 3X3 kernel (central differencing with smoothing).
persistent xKernel;
persistent yKernel;

if (isempty(xKernel) || isempty(yKernel))
    xKernel = [-1/12, 0, 1/12; -4/12, 0, 4/12; -1/12, 0, 1/12];
    yKernel = xKernel';
end

% run kernel over image
slice = double(image((pixel(2)-1):(pixel(2)+1), (pixel(1)-1):(pixel(1)+1)));
grad = [sum(sum(xKernel.*slice)); 
        sum(sum(yKernel.*slice))];

end

