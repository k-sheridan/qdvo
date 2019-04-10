function [warpedPatch] = warpPatchToTargetFrame(landmark, sourceFrame, targetFrame, graph, patchRadius)
%WARPPATCHTOTARGETFRAME This function computes the warped patch to search
%with in the target frame.

%s = Settings();

T_w_sourceImu = sourceFrame.imustate.poseTransform();
T_w_targetImu = targetFrame.imustate.poseTransform();
T_i_c = graph.extrinsics.getImu2CameraTransform(targetFrame.camID);

T_sourceCam_targetCam = (T_w_sourceImu * T_i_c) \ (T_w_targetImu * T_i_c);
T_targetCam_sourceCam = [T_sourceCam_targetCam(1:3, 1:3)', -T_sourceCam_targetCam(1:3, 1:3)' * T_sourceCam_targetCam(1:3, 4);
                         zeros(1, 3), 1];
                     
% find the average normal vector.
pt_source = [landmark.bearing;1] * (1/landmark.dinv);
r_pt_target = T_sourceCam_targetCam(1:3, 4) - pt_source;

normal = (r_pt_target/norm(r_pt_target) + -pt_source/norm(pt_source));
normal = normal / norm(normal);
                     
% compute normal vector in the target frame.
%n = T_targetCam_sourceCam(1:3, 1:3) * landmark.patchNormal;
n = T_targetCam_sourceCam(1:3, 1:3) * normal;
% compute the landmark position in the target frame.
p0 = T_targetCam_sourceCam(1:3, 1:3) * [landmark.bearing;1] * (1/landmark.dinv) + T_targetCam_sourceCam(1:3, 4);
% compute the homogenous bearing to the landmark in the target frame.
if p0(3) <= 1e-10
    error('landmark behind camera.');
end
u0 = p0 / p0(3);

% compute the center pixel position
[px0, projJac] = targetFrame.cameraModel.project(u0);
unprojJac = inv(projJac); % maps small change in pixel position to small change in HOMOGENOUS bearing position.

image = zeros(2*patchRadius + 1);

for dpx = (-patchRadius:patchRadius)
    for dpy = (-patchRadius:patchRadius)
        u = [u0(1:2, 1) + unprojJac * [dpx;dpy]; 1];
        p = T_sourceCam_targetCam(1:3, 1:3) * ((p0'*n) / (u'*n)) * u + T_sourceCam_targetCam(1:3, 4);
        
        try
            px_sample = sourceFrame.cameraModel.project(p);
        catch e
            error('Projection failed: %s\n', e.message);
            return;
        end
        
        try
            brightness = subPixelIntensity1(px_sample, sourceFrame.raw_image);
        catch
            error('Cant compute brightness.');
            return
        end
        
        image(dpy + patchRadius + 1, dpx + patchRadius + 1) = brightness;
        
    end
end

warpedPatch = Patch(image);
                     
end

