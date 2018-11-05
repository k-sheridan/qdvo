function [F, chi2_avg, inlierColumns] = computeFundamentalMatrixRANSAC(bearingVec1, bearingVec2, ransacIters, ransacThreshold, nPoint)
%COMPUTEFUNDAMENTALMATRIX Use ransac to compute the fundamental matrix
%bewteen two feature track vectors.
% feature vector: [[x1_1;y1_1], [x1_2;y1_2], ...]
% for this to work there must be sufficient parallax.
% bearings must obviously be undistorted

% the inlierColumns vector is a boolean vector which says which tracks are
% correct.

% make sure nPoint is at least 8
if (nargin < 5)
    nPoint = 8;
else
    nPoint = max(8, nPoint)
end

[m1, n1] = size(bearingVec1);
[m2, n2] = size(bearingVec2);

assert(m1 == 2 && m2 == 2)
assert(n2 == n1)
assert(n1 > 8)

inlierMatrix = zeros(ransacIters, length(bearingVec1(1, :)));

iterationInfo = {};

for iterationNumber = (1:ransacIters)
    randomColumns = randperm(n1);
    randomColumns = randomColumns(1:nPoint);
    
    % form the correspondence matrix, A, such that A*F = zeros(8, 1)
    A = zeros(nPoint, 9);
    for index = (1:length(randomColumns))
        px1 = bearingVec1(1:2, randomColumns(index));
        px2 = bearingVec2(1:2, randomColumns(index));
        
        xp = px2(1);
        yp = px2(2);
        x = px1(1);
        y = px1(2);
        
        A(index, 1:9) = [x*xp, y*xp, xp, x*yp, y*yp, yp, x, y, 1];
    end
    
    % decompose A and find its null space
    [U, S, V] = svd(A);
    
    fVec = V(1:9, 9)';
    
    F_est = [fVec(1:3); fVec(4:6); fVec(7:9)];
    
    % compute average squared error
    chi2_sum = 0;
    for index = (1:length(bearingVec1(1, :)))
        chi2 = ([bearingVec1(1:2, index);1]' * F_est * [bearingVec2(1:2, index);1])^2;
        chi2_sum = chi2_sum + chi2;
        if (chi2 < ransacThreshold)
            inlierMatrix(iterationNumber, index) = 1;
        end
    end
    
    iterationInfo{end+1} = struct('F', F_est, 'chi2_avg', chi2_sum / length(bearingVec1(1, :)));
    
end

[val, row] = max(sum(inlierMatrix'))

F = iterationInfo{row}.F;
chi2_avg = iterationInfo{row}.chi2_avg;
inlierColumns = inlierMatrix(row, :);

end

