function [warpedPatch] = warpPatchToTargetFrame(landmark, sourceKeyframe, targetKeyframe, patchRadius)
%WARPPATCHTOTARGETFRAME This function will use a homographic transform to
%warp a patch in the source frame into the target frame. This assumes that
%the patch represents a planar feature with known normal and point in
%space.

assert(strcmp(class(landmark), 'Landmark'))
assert(strcmp(class(sourceKeyframe), 'KeyFrame'))
assert(strcmp(class(targetKeyframe), 'KeyFrame'))
assert(strcmp(class(sourceKeyframe.frame), 'Frame'))
assert(strcmp(class(sourceKeyframe.frame.imustate), 'IMUState'))
assert(strcmp(class(sourceKeyframe.frame.cameraModel), 'EquidistantCameraModel'))
assert(strcmp(class(targetKeyframe.frame), 'Frame'))
assert(strcmp(class(targetKeyframe.frame.imustate), 'IMUState'))
assert(strcmp(class(targetKeyframe.frame.cameraModel), 'EquidistantCameraModel'))

% landmark must be in the sourceKFs frame of reference
assert(landmark.kfid == sourceKeyframe.id);

% Definition of Homography
% H = R - t * n' / d

% compute the homography
T_sourceFromWorld = sourceKeyframe.frame.imustate.poseTransform();
T_targetFromWorld = targetKeyframe.frame.imustate.poseTransform();

T_targetFromSource = inv(T_sourceFromWorld) * T_targetFromWorld

R = T_targetFromSource(1:3, 1:3);
t = T_targetFromSource(1:3, 4);
n = landmark.patchNormal;
point = T_sourceFromWorld(1:3, 1:3) * (landmark.bearing * (1/landmark.zinv)) + T_sourceFromWorld(1:3, 4); % point in inertial frame
d = n' * (T_targetFromSource(1:3, 4) - point);

% compute the homography
H = R - t*n' / d;

% project the landmark into the target frame
pointInTarget = T_targetFromWorld(1:3, 1:3)' * point - T_targetFromWorld(1:3, 1:3)' * T_targetFromWorld(1:3, 4);
targetPatchCenterPixel = targetKeyframe.frame.cameraModel.project(pointInTarget)

targetU = zeros(2 * patchRadius + 1);
targetV = targetU;

% expensive step of unprojecting the entire patch.
% Could it be sped up with the unprojection jacobian as an approximation?
% TODO: use the unprojection taylor series expansion for speed.
% The distortion is well approximated by an affine transformation.
for dx = (-patchRadius:patchRadius)
    for dy = (-patchRadius:patchRadius)
        bearing = targetKeyframe.frame.cameraModel.unproject(targetPatchCenterPixel + [dx;dy]);
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

end

