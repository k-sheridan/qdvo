function H = nonMaximalSupp(dx, dy, magGrad, lowThresh, highThresh)
% Perform Non-Maximum Suppression Thining and Hysteresis Thresholding of Edge
% Strength
% H = nonMaximalSupp(dx, dy, magGrad, lowThresh, highThresh)
% H is suppressed image
% dx and dy are the gradients
% magGrad is the magnitude of gradient
% lowThresh and highThresh are the thresholds
magmax = max(magGrad(:));
    if magmax > 0
        magGrad = magGrad / magmax;
    end
[row, col] = size(magGrad);
E = false(row, col);
idxStrong = [];
for dir = 1:4
    idxLocalMax = cannyFindLocalMaxima(dir,dx,dy,magGrad);
    idxWeak = idxLocalMax(magGrad(idxLocalMax) > lowThresh);
    E(idxWeak)=1;
    idxStrong = [idxStrong; idxWeak(magGrad(idxWeak) > highThresh)]; %#ok<AGROW>
end

[m,n] = size(E);

if ~isempty(idxStrong) % result is all zeros if idxStrong is empty
    rstrong = rem(idxStrong-1, m)+1;
    cstrong = floor((idxStrong-1)/m)+1;
    H = bwselect(E, cstrong, rstrong, 8);
else
    H = zeros(m, n);
end