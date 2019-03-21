function [px, p_target, proJac] = projectLandmark(graph, parentFrameID, landmarkID, targetFrameID)
%PROJECTLANDMARK This function will simply project a landmark into a pixel
%position. Also returns the landmark position in the target frame, and the
%projection jacobian.

tf = graph.getFrame(targetFrameID);
sf = graph.getFrame(parentFrameID);
lidx = sf.getLandmarkIndex(landmarkID);

T_i_c = grap.extrinsics.getImu2CameraTransform(tf.camID);
T_w_obsi = tf.imustate.poseTransform();
T_w_sourcei = sf.imustate.poseTransform();

T_obs_source = inv(T_w_obsi * T_i_c) * T_w_sourcei * T_i_c;

p_target = T_obs_source(1:3, 1:3) * [sf.landmarks{lidx}.bearing;1] / sf.landmarks{lidx}.dinv + T_obs_source(1:3, 4);

[px, proJac] = tf.cameraModel.project(p_target);

end

