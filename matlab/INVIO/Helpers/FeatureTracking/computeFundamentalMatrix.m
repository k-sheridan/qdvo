function [F, chi2_avg, inlierColumns] = computeFundamentalMatrixRANSAC(bearingVec1, bearingVec2, ransacIters, ransacThreshold)
%COMPUTEFUNDAMENTALMATRIX Use ransac to compute the fundamental matrix
%bewteen two feature track vectors.
% feature vector: [[x1_1;y1_1], [x1_2;y1_2], ...]
% for this to work there must be sufficient parallax.
% bearings must obviously be undistorted

% the inlierColumns vector is a boolean vector which says which tracks are
% correct.



end

