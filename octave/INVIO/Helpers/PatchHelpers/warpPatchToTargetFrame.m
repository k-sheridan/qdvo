function [warpedPatch] = warpPatchToTargetFrame(landmark, sourceKeyframe, targetKeyframe, patchRadius)
%WARPPATCHTOTARGETFRAME This function will use a homographic transform to
%warp a patch in the source frame into the target frame. This assumes that
%the patch represents a planar feature with known normal and point in
%space.

assert(strcmp(class(landmark), 'Landmark'))
assert(strcmp(class(sourceKeyframe), 'KeyFrame'))
assert(strcmp(class(targetKeyframe), 'KeyFrame'))

% landmark must be in the sourceKFs frame of reference
assert(landmark.kfid == sourceKeyframe.id);

% Definition of Homography
% H = R - t * n' / d

% compute the homography
T_sourceFromWorld = sourceKeyframe.frame.imustate.poseTransform();
T_targetFromWorld = targetKeyframe.frame.imustate.poseTransform();

T_targetFromSource = inv(T_sourceFromWorld) * T_targetFromWorld;

R = T_targetFromSource(1:3, 1:3);
t = T_targetFromSource(1:3, 4);
n = landmark.patchNormal;
point = T_sourceFromWorld * [landmark.bearing; 1] * (1/landmark.zinv); % point in inertial frame
d = n' * (T_targetFromSource(1:3, 4) - point);

% compute the homography
H = R - t*n' / d;


end

