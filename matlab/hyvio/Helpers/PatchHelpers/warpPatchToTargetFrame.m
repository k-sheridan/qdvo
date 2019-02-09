function [warpedPatch, error] = warpPatchToTargetFrame(landmark, sourceKeyframe, targetKeyframe, patchRadius)
%WARPPATCHTOTARGETFRAME This function will use a homographic transform to
%warp a patch in the source frame into the target frame. This assumes that
%the patch represents a planar feature with known normal and point in
%space.

error = 0;

assert(isa(landmark, 'Landmark'))
assert(isa(sourceKeyframe, 'Frame'))
assert(isa(targetKeyframe, 'Frame'))
assert(isa(sourceKeyframe.imustate, 'IMUState'))
assert(isa(sourceKeyframe.cameraModel, 'EquidistantCameraModel'))
assert(isa(targetKeyframe.imustate, 'IMUState'))
assert(isa(targetKeyframe.cameraModel, 'EquidistantCameraModel'))

% landmark must be in the sourceKFs frame of reference
assert(landmark.frameID == sourceKeyframe.ID);

% Definition of Homography
% H = R - t * n' / d

% compute the homography
T_sourceFromWorld = sourceKeyframe.imustate.poseTransform();
T_targetFromWorld = targetKeyframe.imustate.poseTransform();

T_targetFromSource = (T_sourceFromWorld) \ T_targetFromWorld;

R = T_targetFromSource(1:3, 1:3);
t = T_targetFromSource(1:3, 4);
n = landmark.patchNormal;
point = T_sourceFromWorld(1:3, 1:3) * ([landmark.bearing;1] * (1/landmark.dinv)) + T_sourceFromWorld(1:3, 4); % point in inertial frame
d = n' * (T_targetFromSource(1:3, 4) - point);

% compute the homography
H = R - t*n' / d;

% project the landmark into the target frame
pointInTarget = T_targetFromWorld(1:3, 1:3)' * point - T_targetFromWorld(1:3, 1:3)' * T_targetFromWorld(1:3, 4);
targetPatchCenterPixel = targetKeyframe.cameraModel.project(pointInTarget);

targetU = zeros(2 * patchRadius + 1);
targetV = targetU;

% expensive step of unprojecting the entire patch.
% Could it be sped up with the unprojection jacobian as an approximation?
% TODO: use the unprojection taylor series expansion for speed.
% The distortion is well approximated by an affine transformation.
[centerBearing, error, unprojectJacobian] = targetKeyframe.cameraModel.unproject(targetPatchCenterPixel);

for dx = (-patchRadius:patchRadius)
    for dy = (-patchRadius:patchRadius)
        
        bearing = centerBearing(1:2) + unprojectJacobian * [dx; dy];
        
        row = dy + patchRadius + 1;
        col = dx + patchRadius + 1;
        targetU(row, col) = bearing(1);
        targetV(row, col) = bearing(2);
    end
end

% Plot for paper
%figure
%scatter(reshape(targetU, [numel(targetU), 1]), reshape(targetV, [numel(targetU), 1]), 'ks')
%title('Undistorted and Normalized Patch')
%xlabel('u (m)')
%ylabel('v (m)')

% transform the normalized coordinates with the homography and project them
% to pixels in the source frame. Finally, evaluate the source image at those
% pixels.

maxIndex = 2 * patchRadius + 1;
warpedImageData = zeros(maxIndex);

centerRow = patchRadius + 1;
centerCol = centerRow;
centerBearing = H*[targetU(centerRow, centerCol); targetV(centerRow, centerCol); 1];
[centerPx, error, projectJacobian] = sourceKeyframe.cameraModel.project(centerBearing);


for row = (1:maxIndex)
    for col = (1:maxIndex)
        sourceBearing = H*[targetU(row, col); targetV(row, col); 1];
        px = centerPx + projectJacobian * [sourceBearing(1:2) - centerBearing(1:2)];
        
        try
            intensity = subPixelIntensity1(px, sourceKeyframe.raw_image);
        catch
            disp('pixel out of bounds in source.')
            warpedPatch = Patch(warpedImageData);
            error = 1;
            return;
        end
        
        warpedImageData(row, col) = intensity;
    end
end

warpedPatch = Patch(warpedImageData);
end

